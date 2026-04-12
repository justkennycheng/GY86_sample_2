#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// ---------------------------
// 硬件 & DMP 相关
// ---------------------------
MPU6050 mpu;

bool dmpReady = false;      // DMP 是否初始化成功
uint8_t mpuIntStatus;       // 中断状态
uint8_t devStatus;          // DMP 初始化状态
uint16_t packetSize;        // 每个 DMP 包的字节数
uint16_t fifoCount;         // FIFO 当前字节数
uint8_t fifoBuffer[64];     // FIFO 缓冲区

bool easymode_enable = ture;   //默认开启PID外环

// ---------------------------
// 姿态相关（四元数 + YPR）
// ---------------------------
Quaternion q_current;               // 当前姿态四元数
Quaternion q_target;               //  目标姿态四元数
VectorFloat gravity;        // 重力向量
float ypr[3];               // yaw, pitch, roll（单位：弧度）

//陀螺仪数据
int16_t gx_raw, gy_raw, gz_raw;

// ----------- PID 状态变量 -----------
float roll_i = 0.0f; float roll_last = 0.0f;
float pitch_i = 0.0f; float pitch_last = 0.0f;
float yaw_i = 0.0f; float yaw_last = 0.0f;


void setup() {
    Serial.begin(115200);
    delay(1000);

    // ESP32-C3 I2C 引脚：SDA=8, SCL=10
    Wire.begin(8, 10);
    Wire.setClock(400000);

    Serial.println("Initializing I2C devices...");
    mpu.initialize();

    Serial.println("Testing device connections...");
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    Serial.println("Initializing DMP...");
    devStatus = mpu.dmpInitialize();

    // 根据你的实际安装方向做轴/方向校准（这里先用默认值）
    // mpu.setXAccelOffset(...);
    // mpu.setYAccelOffset(...);
    // mpu.setZAccelOffset(...);
    // mpu.setXGyroOffset(...);
    // mpu.setYGyroOffset(...);
    // mpu.setZGyroOffset(...);

    if (devStatus == 0) {
        // 开启 DMP
        mpu.setDMPEnabled(true);
        // 获取中断状态
        mpuIntStatus = mpu.getIntStatus();
        // 获取 DMP 包大小
        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;
        Serial.println("DMP ready! Waiting for data...");
    } else {
        Serial.print("DMP Initialization failed (code ");
        Serial.print(devStatus);
        Serial.println(")");
    }
}

void loop() {
    //计算dt
    static uint32_t last_t = micros();      //微秒
    uint32_t now_t = micros();
    float dt = (now_t - last_t) * 1e-6f;    // 转换成秒
    last_t = now_t;

    if (!dmpReady) return;
    // 检查 FIFO 中是否有完整包
    fifoCount = mpu.getFIFOCount();
    if (fifoCount < packetSize) {
        // 数据还不够一包，稍等
        return;
    }
    if (fifoCount >= 1024) {
        // FIFO 溢出，清空
        mpu.resetFIFO();
        Serial.println("FIFO overflow!");
        return;
    }
    // 读取一个完整的 DMP 包
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // 从 DMP 包中解析四元数和重力向量
    mpu.dmpGetQuaternion(&q_current, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q_current);
    mpu.dmpGetYawPitchRoll(ypr, &q_current, &gravity);

    // ypr[0] = yaw, ypr[1] = pitch, ypr[2] = roll（单位：弧度）
    float yaw   = rad2deg(ypr[0]);
    float pitch = rad2deg(ypr[1]);
    float roll  = rad2deg(ypr[2]);

    //获取陀螺仪数据
    mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);
    // 从LSB 转换为 deg/s
    float gx = gx_raw / 16.4f;
    float gy = gy_raw / 16.4f;
    float gz = gz_raw / 16.4f;
    // 转换为 rad/s（推荐）
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    //摇杆输入转四元数
    // 1. 摇杆输入（-1.0 ~ +1.0）
    float rc_roll;   // 左右
    float rc_pitch;  // 前后
    float rc_yaw;    // 偏航
    // 2. 最大姿态角（你可以调）
    float max_roll  = 30.0f * 0.0174533f;   //转换为 rad/s
    float max_pitch = 30.0f * 0.0174533f;   //转换为 rad/s
    // 3. roll/pitch 目标角度
    float roll_target  = rc_roll  * max_roll;
    float pitch_target = rc_pitch * max_pitch;
    // 4. yaw 是累积角度（航向锁定）
    static float yaw_target = 0.0f;
    yaw_target += rc_yaw * 0.02f;   // 每次循环累积一点
    // 5. 欧拉角 → 四元数
    q_target = eulerToQuaternion( roll_target, pitch_target, yaw_target );


    // q_current 来自 DMP
    // q_target 由你的模式决定（例如水平 = (1,0,0,0)）
    // gx,gy,gz 来自陀螺仪（注意单位统一）
    // dt 是循环时间
    float u_roll, u_pitch, u_yaw;
    attitudeControlStep(q_target, q_current, gx, gy, gz, dt, u_roll, u_pitch, u_yaw);

    // 这里先不做 motorMix，等你准备好了我们再接下去



    //打印部分
    // ---------------------------
    // 航空航天右手坐标系约定：
    //  - roll > 0  : 右滚（右机翼下沉）
    //  - pitch > 0 : 机头抬起（nose up）
    //  - yaw > 0   : 机头向右偏航
    // Jeff Rowberg 的 dmpGetYawPitchRoll 本身就是这个约定
    // ---------------------------
    Serial.printf("dt=%.4fs  YAW=%.2fdeg  PITCH=%.2fdeg  ROLL=%.2fdeg  GX=%.2frad/s  GY=%.2frad/s  GZ=%.2frad/s\n",
              dt, yaw, pitch, roll, gx, gy, gz);
    // 这里你可以直接把 roll/pitch/yaw 喂给姿态 PID
    // 注意：控制时用“角度误差（圆上最短差）”，不要直接相减

}

////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////

// ---------------------------
// 工具函数：弧度转角度
// ---------------------------
inline float rad2deg(float r) {
    return r * 180.0f / M_PI;
}

//四元数串级 PID
void attitudeControlStep(
    const Quaternion& q_target,
    const Quaternion& q_current,
    float gx, float gy, float gz,
    float dt,
    float& u_roll, float& u_pitch, float& u_yaw)
{
    // 1. 四元数姿态误差
    Quaternion q_err = q_target * q_current.getConjugate();

    // 2. 姿态误差向量（目标角速度）
    float ex = 2.0f * q_err.x;
    float ey = 2.0f * q_err.y;
    float ez = 2.0f * q_err.z;

    // 3. 角速度误差
    if (easymode_enable){
        float err_roll_rate  = ex - gx;
        float err_pitch_rate = ey - gy;
        float err_yaw_rate   = ez - gz;
    }else{
        //禁用外环的情况下,角速度误差直接使用摇杆输入
        float err_roll_rate  = target_roll_rate  - gx;      //target_roll_rate的产生应该有一个系数,后面再加上
        float err_pitch_rate = target_pitch_rate - gy;
        float err_yaw_rate   = target_yaw_rate   - gz;
    }
    
    
    // 4. Rate PID（函数版）
    u_roll = pid_update(err_roll_rate, dt,
                        roll_i, roll_last,
                        0.8f, 0.0f, 0.02f,  //kp,ki,kd
                        -500.0f, 500.0f);

    u_pitch = pid_update(err_pitch_rate, dt,
                         pitch_i, pitch_last,
                         0.8f, 0.0f, 0.02f,  //kp,ki,kd
                         -500.0f, 500.0f);

    u_yaw = pid_update(err_yaw_rate, dt,
                       yaw_i, yaw_last,
                       0.6f, 0.0f, 0.00f,  //kp,ki,kd
                       -500.0f, 500.0f);
}

//pid算法
float pid_update(float err, float dt,
                 float &i_term, float &last_err,
                 float kp, float ki, float kd,
                 float out_min, float out_max)
{
    // 积分
    i_term += err * dt;

    // 抗积分饱和
    if (i_term > out_max) i_term = out_max;
    if (i_term < out_min) i_term = out_min;

    // 微分
    float d = (err - last_err) / dt;
    last_err = err;

    // PID 输出
    float out = kp * err + ki * i_term + kd * d;

    // 输出限幅
    if (out > out_max) out = out_max;
    if (out < out_min) out = out_min;

    return out;
}

//欧拉角 → 四元数函数
Quaternion eulerToQuaternion(float roll, float pitch, float yaw)
{
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    Quaternion q;
    q.w = cr*cp*cy + sr*sp*sy;
    q.x = sr*cp*cy - cr*sp*sy;
    q.y = cr*sp*cy + sr*cp*sy;
    q.z = cr*cp*sy - sr*sp*cy;
    return q;
}


