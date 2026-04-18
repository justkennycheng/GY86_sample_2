#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// ---------------------------
// 硬件 & DMP 相关
MPU6050 mpu;

bool dmpReady = false;      // DMP 是否初始化成功
uint8_t mpuIntStatus;       // 中断状态
uint8_t devStatus;          // DMP 初始化状态
uint16_t packetSize;        // 每个 DMP 包的字节数
uint16_t fifoCount;         // FIFO 当前字节数
uint8_t fifoBuffer[64];     // FIFO 缓冲区

bool easymode_enable = true;   //默认开启PID外环

// 姿态相关（四元数 + YPR）
Quaternion q_current;               // 当前姿态四元数
Quaternion q_target;               //  目标姿态四元数
VectorFloat gravity;        // 重力向量
float ypr[3];               // yaw, pitch, roll（单位：弧度）

//陀螺仪数据
int16_t gx_raw, gy_raw, gz_raw;

// ----------- PID 状态存储变量 -----------
float roll_i = 0.0f; float roll_last = 0.0f;
float pitch_i = 0.0f; float pitch_last = 0.0f;
float yaw_i = 0.0f; float yaw_last = 0.0f;


//最大姿态角（可调）
float max_roll  = 40.0f * 0.0174533f;   //转换为 rad
float max_pitch = 40.0f * 0.0174533f;   //转换为 rad

//油门摇杆输入//1200-1800
float rc_thr = 1500.0;     
//摇杆输入//-1至1 
float rc_roll = 0.0;   // 左右
float rc_pitch =  0.0;  // 前后
float rc_yaw =  0.0;    // 偏航



////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////

void setup() {
    Serial.begin(115200);
    delay(1000);

    // ESP32-C3 I2C 引脚：SDA=8, SCL=10
    Wire.begin(18, 19);
    Wire.setClock(400000);

    Serial.println("Initializing I2C devices...");
    mpu.initialize();

    //设置时钟源（飞控必做）,切换到 陀螺仪 PLL 时钟(若不设置, MPU6050 默认使用内部 8MHz RC 振荡器)
    mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);

    //设置陀螺仪量程,飞控一般用 ±1000°/s 或 ±500°/s
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);

    //设置加速度计量程,默认 ±2g，太小，飞控一般用 ±4g 或 ±8g
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);

    //设置硬件低通滤波（DLPF）
    mpu.setDLPFMode(MPU6050_DLPF_BW_98);   // 或 42Hz

    //设置采样率（可选但推荐）
    mpu.setRate(4);   // 200Hz

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

    delay(1000);
    calibrateGyro();  // ⭐ 自动校准

}

void loop() {
    //计算dt
    static uint32_t last_t = micros();      //微秒
    uint32_t now_t = micros();
    float dt = (now_t - last_t) * 1e-6f;    // 转换成秒
    last_t = now_t;
    
    /////////////获得四元数和陀螺仪角速度/////////////////
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
    gx += 0.02f;       /////////校准后还是总偏-0.02

    ///////////根据摇杆输入构造q_target///////////////////////////
    //摇杆输入转四元数
    //roll/pitch 目标角度
    float roll_target  = rc_roll  * max_roll;   //摇杆输入（-1.0 ~ +1.0）转为角度(rad)
    float pitch_target = rc_pitch * max_pitch;  //摇杆输入（-1.0 ~ +1.0）转为角度(rad)
    //yaw 是累积角度（航向锁定）
    static float yaw_target = 0.0f;
    yaw_target += rc_yaw * 0.02f;   // 每次循环累积一点
    //欧拉角 → 四元数
    q_target = eulerToQuaternion(roll_target, pitch_target, yaw_target);

    ///////////进行PID计算得到姿态控制输出////////////////////////
    float err_pitch_rate, err_roll_rate, err_yaw_rate, u_roll, u_pitch, u_yaw;
    attitudeControlStep(q_target, q_current, gx, gy, gz, dt, err_pitch_rate, err_roll_rate, err_yaw_rate, u_roll, u_pitch, u_yaw);   //得到姿态控制输出u_roll, u_pitch, u_yaw, 这三个值似乎是机体坐标系.

    //电机混控motorMix
    float m1, m2, m3, m4;
    motorMix(rc_thr, u_roll, u_pitch, u_yaw, m1, m2, m3, m4);     //m输出范围1000-2000

    //打印部分, 定期执行
    static uint32_t lastTick = 0;
    if (millis() - lastTick > 5) {
        //Serial.printf("dt=%.4fs  YAW=%.2fdeg  PITCH=%.2fdeg  ROLL=%.2fdeg  gx=%.2frad/s  gy=%.2frad/s  gz=%.2frad/s  m1=%.1f m1=%.1f m1=%.1f m1=%.1f\n",
        //      dt, yaw, pitch, roll, gx, gy, gz, m1, m2, m3, m4);
        //Serial.printf("yaw:%.2f pitch:%.2f roll:%.2f ", yaw, pitch, roll);
        //Serial.printf("qw:%.5f qx:%.5f qy:%.5f qz:%.5f ",q_current.w, q_current.x, q_current.y, q_current.z);
        //Serial.printf("gx:%.2f gy:%.2f gz:%.2f ", gx, gy, gz);
        //Serial.printf("pitch:%.2f roll:%.2f yaw:%.2f gx:%.2f gy:%.2f gz:%.2f m1:%.1f m2=:%.1f m3:%.1f m4:%.1f", pitch, roll, yaw, gx, gy, gz, m1, m2, m3, m4);
        Serial.printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
             pitch, roll, yaw, gx, gy, gz, err_pitch_rate, err_roll_rate, err_yaw_rate, u_roll, u_pitch, u_yaw, m1, m2, m3, m4);
        ///
        //Serial.printf("%.6f %.6f %.6f %.6f",q_current.w, q_current.x, q_current.y, q_current.z);
        lastTick = millis();
    }
    

    //Serial.print("\n");
}

////////////////////////////////////////////////////
//////////////////FUNCTION//////////////////////////
////////////////////////////////////////////////////
// ---------------------------


//四元数串级 PID
void attitudeControlStep(
    Quaternion& q_target,
    Quaternion& q_current,
    float gx, float gy, float gz,
    float dt,
    float& err_pitch_rate, float& err_roll_rate, float& err_yaw_rate,
    float& u_roll, float& u_pitch, float& u_yaw)
{
    // 1. 四元数姿态误差
    Quaternion q_err = q_target.getProduct(q_current.getConjugate());

    // 2. 姿态误差向量（目标角速度）
    float ex = 2.0f * q_err.x;  //sin(θ/2) 最大是 1，所以 ex 理论最大可以到 2. 
    float ey = 2.0f * q_err.y;
    float ez = 2.0f * q_err.z;
    //Serial.printf("%.4f,%.4f,%.4f ",ex, ey, ez);
    err_roll_rate = 0.0;
    err_pitch_rate = 0.0;
    err_yaw_rate  = 0.0;


    // 3. 角速度误差
    if (easymode_enable){
        //PID外环启用
        err_roll_rate  = ex * 5.0f - gx;   //ex的范围在[-2.0, 2.0]之间,要给 ex 乘以一个比例系数。
        err_pitch_rate = ey * 5.0f - gy;
        err_yaw_rate   = ez * 5.0f - gz;
    }else
    {
        //禁用外环的情况下,角速度误差直接使用摇杆输入
        float target_roll_rate  = rc_roll  * 7.0f;   //将摇杆输入(-1至1)乘以一个适当的系数以便和角速度相减
        float target_pitch_rate = rc_pitch * 7.0f;
        float target_yaw_rate   = rc_yaw   * 7.0f;

        err_roll_rate  = target_roll_rate  - gx;      // 实际飞行中通常在 0-10 rad/s波动，极限可达±35rad/s
        err_pitch_rate = target_pitch_rate - gy;
        err_yaw_rate   = target_yaw_rate   - gz;
    }
    //Serial.printf("%.4f,%.1f,%.1f\n",err_roll_rate, err_pitch_rate, err_yaw_rate);
    
    // 4. Rate PID（函数版）
    u_roll = pid_update(err_roll_rate, dt,
                        roll_i, roll_last,
                        15.0f, 0.1f, 0.02f,  //kp,ki,kd
                        -300.0f, 300.0f);

    u_pitch = pid_update(err_pitch_rate, dt,
                         pitch_i, pitch_last,
                         15.0f, 0.1f, 0.02f,  //kp,ki,kd
                         -300.0f, 300.0f);

    u_yaw = pid_update(err_yaw_rate, dt,
                       yaw_i, yaw_last,
                       10.0f, 0.0f, 0.00f,  //kp,ki,kd
                       -150.0f, 150.0f);
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
    if (i_term > out_max/4.0f) i_term = out_max/4.0f;     //积分限幅同
    if (i_term < out_min/4.0f) i_term = out_min/4.0f;

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




void motorMix(float throttle,
              float u_roll, float u_pitch, float u_yaw,
              float &m1, float &m2, float &m3, float &m4)
{
    m1 = throttle - u_roll - u_pitch - u_yaw;  // 右前 CCW
    m2 = throttle + u_roll - u_pitch + u_yaw;  // 左前 CW
    m3 = throttle + u_roll + u_pitch - u_yaw;  // 左后 CCW
    m4 = throttle - u_roll + u_pitch + u_yaw;  // 右后 CW

    // 限幅
    m1 = constrain(m1, 1000, 2000);
    m2 = constrain(m2, 1000, 2000);
    m3 = constrain(m3, 1000, 2000);
    m4 = constrain(m4, 1000, 2000);
}

//陀螺仪自动校准代码
void calibrateGyro() {
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int16_t gx, gy, gz;

    Serial.println("Calibrating gyro... Keep the device absolutely still!");

    // 采样次数
    const int samples = 2000;

    for (int i = 0; i < samples; i++) {
        mpu.getRotation(&gx, &gy, &gz);
        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;
        delay(2);  // 500 Hz 采样
    }

    int16_t gx_offset = -(gx_sum / samples);
    int16_t gy_offset = -(gy_sum / samples);
    int16_t gz_offset = -(gz_sum / samples);

    // 写入 offset
    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);

    Serial.println("Gyro calibration done!");
    Serial.print("Offsets: ");
    Serial.print(gx_offset); Serial.print(", ");
    Serial.print(gy_offset); Serial.print(", ");
    Serial.println(gz_offset);
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

// 工具函数：弧度转角度
inline float rad2deg(float r) {
    return r * 180.0f / M_PI;
}
