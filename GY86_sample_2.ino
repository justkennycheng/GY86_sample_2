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


////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////

void setup() {
    Serial.begin(115200);
    delay(1000);

    // ESP32-C3 MINI I2C 引脚：SDA=8, SCL=9
    Wire.begin(8, 9);
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
    calibrateGyro();  //自动校准陀螺仪零漂

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
    //gx += 0.02f;       /////////校准后还是总偏-0.02


    //打印部分, 定期执行
    static uint32_t lastTick = 0;
    if (millis() - lastTick > 5) {

        Serial.printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
             pitch, roll, yaw, gx, gy, gz);

        lastTick = millis();
    }
    

}

////////////////////////////////////////////////////
//////////////////FUNCTION//////////////////////////
////////////////////////////////////////////////////
// ---------------------------


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

// 工具函数：弧度转角度
inline float rad2deg(float r) {
    return r * 180.0f / M_PI;
}