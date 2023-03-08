#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "CSerialPort/SerialPort.h"
#include "FashionStar/UServo/FashionStar_UartServoProtocol.h"
#include "FashionStar/UServo/FashionStar_UartServo.h"

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

class uservo_move
{
private:
    // 创建NodeHandle
    ros::NodeHandle nh;
    // 创建接收者
    ros::Subscriber set_servo_angle_sub;
    //创建发送者
    ros::Publisher target_pub;

public:
    uint16_t interval; // 运行周期 单位ms
    uint16_t t_acc;    // 加速时间 单位ms
    uint16_t t_dec;    // 减速时间 单位ms
    float velocity;    // 目标转速 单位°/s
    std_msgs::Bool target_flag;
    
    uservo_move();
    void set_servo_angle(float angle0, float angle1);
    void init_servo_angle(float angle0, float angle1);
    void waitAndReport(fsuservo::FSUS_Servo *servo0, fsuservo::FSUS_Servo *servo1);

};

uservo_move::uservo_move()
{
    ros::Duration du(3);
    target_flag.data = 0;
    target_pub = nh.advertise<std_msgs::Bool>("target_flag", 10);
    target_pub.publish(target_flag);

    init_servo_angle(0, 0);
    du.sleep();

    set_servo_angle(-10, -10);
    target_flag.data = 1;
    target_pub.publish(target_flag);
    du.sleep();
    target_flag.data = 0;
    target_pub.publish(target_flag);
    
    set_servo_angle(10, -10);
    target_flag.data = 1;
    target_pub.publish(target_flag);
    du.sleep();
    target_flag.data = 0;
    target_pub.publish(target_flag);

    set_servo_angle(-10, 25);
    target_flag.data = 1;
    target_pub.publish(target_flag);
    du.sleep();
    target_flag.data = 0;
    target_pub.publish(target_flag);

    set_servo_angle(10, 25);
    target_flag.data = 1;
    target_pub.publish(target_flag);
    du.sleep();
    target_flag.data = 0;
    target_pub.publish(target_flag);
    
    init_servo_angle(0, 0);
}

void uservo_move::waitAndReport(fsuservo::FSUS_Servo *servo0, fsuservo::FSUS_Servo *servo1)
{
    ros::Duration(1).sleep();
    servo0->wait(); // 等待舵机旋转到目标角度
    servo1->wait();
    cout << "servo ID = 0, Real Angle = " << servo0->curRawAngle << ", Target Angle = " << servo0->targetRawAngle << endl;
    cout << "servo ID = 1, Real Angle = " << servo1->curRawAngle << ", Target Angle = " << servo1->targetRawAngle << endl;
    
    // protocol.delay_ms(2000); // 暂停2s
}

void uservo_move::set_servo_angle(float angle0, float angle1)
{
    velocity = 30.0; // 设置舵机速度
    t_acc = 25;
    t_dec = 25;

    ROS_INFO("set Angle0 = %f, Angle1 = %f", angle0, angle1);
    servo0.setRawAngleMTurnByVelocity(angle0, velocity, t_acc, t_dec, 0);
    servo1.setRawAngleMTurnByVelocity(angle1, velocity, t_acc, t_dec, 0);
    waitAndReport(&servo0, &servo1);
}

void uservo_move::init_servo_angle(float angle0, float angle1)
{
    velocity = 50.0; // 设置舵机速度
    t_acc = 25;
    t_dec = 25;

    ROS_INFO("initialize Angle0 = %f, Angle1 = %f", angle0, angle1);
    servo0.setRawAngleMTurnByVelocity(angle0, velocity, t_acc, t_dec, 0);
    servo1.setRawAngleMTurnByVelocity(angle1, velocity, t_acc, t_dec, 0);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "uservo_move");
    while (ros::ok())
    {
        uservo_move move;
    }
    
    ros::spin();

    return 0;
}
