#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "CSerialPort/SerialPort.h"
#include "FashionStar/UServo/FashionStar_UartServoProtocol.h"
#include "FashionStar/UServo/FashionStar_UartServo.h"

#include "Car_Spraying/SetTwoAngle.h"
// 串口总线舵机配置
// 设置串口总线Servo转接板的端口号
#ifdef _WIN32
#define SERVO_PORT_NAME "COM8" // Windows下端口号名称 COM{}
#else
#define SERVO_PORT_NAME "/dev/ttyUSB0" // Linux下端口号名称 /dev/ttyUSB{}
#endif
#define DAMPING_POWER 800 // 阻尼模式的功率

#define SERVO_ID_0 0 // 底部舵机ID号
#define SERVO_ID_1 1 // 上部舵机ID号

// 创建协议对象
fsuservo::FSUS_Protocol protocol(SERVO_PORT_NAME, FSUS_DEFAULT_BAUDRATE);
// 创建一个舵机对象
fsuservo::FSUS_Servo servo0(SERVO_ID_0, &protocol);
fsuservo::FSUS_Servo servo1(SERVO_ID_1, &protocol);

uint16_t t_acc = 50;   // 加速时间 单位ms
uint16_t t_dec = 50;   // 减速时间 单位ms
float velocity = 25.0; // 目标转速 单位°/s

void waitAndReport(fsuservo::FSUS_Servo *servo0, fsuservo::FSUS_Servo *servo1)
{
    servo0->wait(); // 等待舵机旋转到目标角度
    servo1->wait();
    cout << "servo ID = 0, Real Angle = " << servo0->curRawAngle << ", Target Angle = " << servo0->targetRawAngle << endl;
    cout << "servo ID = 1, Real Angle = " << servo1->curRawAngle << ", Target Angle = " << servo1->targetRawAngle << endl;
}

void set_servo_angle_callback(const Car_Spraying::SetTwoAngle &data)
{
    // ROS_INFO("Set Angle0 = %f, Angle1 = %f", data.angle0, data.angle1);
    servo0.setRawAngleMTurnByVelocity(data.angle0, velocity, t_acc, t_dec, 0);
    servo1.setRawAngleMTurnByVelocity(data.angle1, velocity, t_acc, t_dec, 0);
    // waitAndReport(&servo0, &servo1);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "uservo_move_feedback");
    // 创建NodeHandle
    ros::NodeHandle nh;
    // 创建接收者
    ros::Subscriber set_servo_angle_sub;
    //创建发送者
    ros::Publisher target_pub, servo_angle_pub;
    //舵机真实角度
    Car_Spraying::SetTwoAngle Angle_data;

    servo_angle_pub = nh.advertise<Car_Spraying::SetTwoAngle>("/servo_angle", 10);

    ros::Rate rate(10);
    while (ros::ok())
    {
        // 通过SDK查询舵机角度
        Angle_data.angle0 = servo0.queryRawAngle();
        Angle_data.angle1 = servo1.queryRawAngle();
        ROS_INFO("Real Angle0: %f Angle1: %f", Angle_data.angle0, Angle_data.angle1);

        set_servo_angle_sub = nh.subscribe("/set_servo_angle", 2, set_servo_angle_callback);
        servo_angle_pub.publish(Angle_data);
        rate.sleep();
        ros::spinOnce();
    }

    // ros::spin();
    return 0;
}