/*
 * 舵机控制节点(Demo)
 */
// 导入ROS依赖
#include "ros/ros.h"
#include "Car_Spraying/SetServoAngle.h"
#include "Car_Spraying/SetServoDamping.h"
#include "Car_Spraying/QueryServoAngle.h"
#include "Car_Spraying/SetTwoAngle.h"

#include "CSerialPort/SerialPort.h"
#include "FashionStar/UServo/FashionStar_UartServoProtocol.h"
#include "FashionStar/UServo/FashionStar_UartServo.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf/transform_broadcaster.h>
#include "math.h"
using namespace fsuservo;
using namespace Car_Spraying;

// 参数定义
#define SERVO_PORT_NAME "/dev/ttyUSB0" // Linux下端口号名称 /dev/ttyUSB{}
#define SERVO_ID0 0                    // 舵机ID号
#define SERVO_ID1 1                    // 舵机ID号
#define PI 3.141592653
#define Height 0.05723
#define x_offset 0.0106
#define y_offset 0.0175
#define z_offset 0.0125
//相机内参
const double fx = 612.795;
const double fy = 612.789;
const double cx = 325.436;
const double cy = 236.364;

// 创建协议对象
FSUS_Protocol protocol(SERVO_PORT_NAME, FSUS_DEFAULT_BAUDRATE);
// 创建一个舵机对象
FSUS_Servo servo0(SERVO_ID0, &protocol);
FSUS_Servo servo1(SERVO_ID1, &protocol);


/* 舵机角度设置回调函数 */
void set_servo_angle_callback(const SetTwoAngle &data)
{
    // ROS_INFO("[RECV] Servo ID = %d Set Angle = %.1f", data.id, data.angle);
    // 设置舵机角度
    // servo0.setRawAngle(data.angle, 0);
    float velocity = 50.0; // 设置舵机速度
    int t_acc = 25;
    int t_dec = 25;
    // servo0.setAngle(data.angle0);
    // servo1.setAngle(data.angle1);
    servo0.setRawAngleMTurnByVelocity(data.angle0, velocity, t_acc, t_dec, 0);
    servo1.setRawAngleMTurnByVelocity(data.angle1, velocity, t_acc, t_dec, 0);
    // float now_angle = 0.0;
    // now_angle = servo0.queryRawAngle();
    // ROS_INFO("curRawAngle = %.2f ",now_angle);
}

/* 舵机阻尼模式回调函数 */
void set_servo_damping_callback(const SetServoDamping &data)
{
    // ROS_INFO("[RECV] Servo ID = %d, Set Damping Power = %d", data.id, data.power);
    servo0.setDamping(data.power);
}

/* 舵机角度查询 */
bool query_servo_angle_callback(QueryServoAngle::Request &req, QueryServoAngle::Response &res)
{
    float angle0, angle1;
    // 通过SDK查询舵机角度
    angle0 = servo0.queryRawAngle();
    // 填写返回数据包
    res.angle = angle0;
    // 发布TF
    // static tf2_ros::TransformBroadcaster broadcaster;
    // geometry_msgs::TransformStamped ts;
    // ts.header.frame_id="servo_link";
    // ts.header.stamp=ros::Time::now();
    // ts.child_frame_id="camera_link";
    // ts.transform.translation.x=0.0106;
    // ts.transform.translation.y=0.0175;
    // ts.transform.translation.z=0.05723;
    // tf2::Quaternion qtn;
    // qtn.setRPY(0,0,angle0 * PI / 180);
    // ts.transform.rotation.x=qtn.getX();
    // ts.transform.rotation.y=qtn.getY();
    // ts.transform.rotation.z=qtn.getZ();
    // ts.transform.rotation.w=qtn.getW();
    // broadcaster.sendTransform(ts);
    static tf::TransformBroadcaster br; //广播器
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0106, 0.0175, 0.05723)); //设置平移部分

    //从IMU消息包中获取四元数数据
    tf::Quaternion q;
    // tf2::Quaternion qtn;
    q.setRPY(0, 0, angle0 * PI / 180);
    // q.setX(imu_data->orientation.x);
    // q.setY(imu_data->orientation.y);
    // q.setZ(imu_data->orientation.z);
    // q.setW(imu_data->orientation.w);
    q.normalized(); //归一化

    transform.setRotation(q); //设置旋转部分
    //广播出去
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "servo_link", "camera_link"));

    // 打印日志
    ROS_INFO("[RECV] Servo ID = %d  , Query Servo Angle = %.2f", req.id, angle0);
    // 注: C++版本的 服务回调函数，最后一定要返回一个bool值
    return true;
}

std::vector<float> TranDistence(float theta_x, float theta_y, float theta_z)
{
    theta_x = PI * theta_x / 180;
    theta_y = -PI * theta_y / 180;
    theta_z = PI * theta_z / 180;
    std::vector<float> distence;

    //计算绕y轴旋转平移量
    float r_1 = sqrt(pow(x_offset, 2) + pow(Height, 2));
    float theta_1 = atan(Height / x_offset); //弧度
    float z_1 = r_1 * sin(theta_1 + theta_y);
    float x_1 = r_1 * cos(theta_1 + theta_y);
    float delta_z = -(Height - z_1);
    float delta_x_1 = x_1 - x_offset;

    //计算绕z轴旋转平移量
    float r_2 = sqrt(pow(x_1, 2) + pow(y_offset, 2));
    float theta_2 = atan(y_offset / x_1);
    if (theta_2 >= 0)
        theta_2 = theta_2;
    else
        theta_2 = PI + theta_2;
    float x_2 = r_2 * cos(theta_z + theta_2);
    float y_2 = r_2 * sin(theta_z + theta_2);
    float delta_y = y_2 - y_offset;
    float delta_x_2 = x_2 - x_1;
    float delta_x = delta_x_1 + delta_x_2;

    // std::cout << "delta_x: " << delta_x << std::endl;
    // std::cout << "delta_y: " << delta_y << std::endl;
    // std::cout << "delta_z: " << delta_z << std::endl;

    distence.push_back(delta_x);
    distence.push_back(delta_y);
    distence.push_back(delta_z);
    return distence;
}


int main(int argc, char **argv)
{
    // 创建节点名称
    ros::init(argc, argv, "uservo_demo_node");
    // 创建NodeHandle
    ros::NodeHandle node_handle;

    // 创建接收者
    ros::Subscriber set_servo_angle_sub = node_handle.subscribe("set_servo_angle", 2, set_servo_angle_callback);
    ros::Subscriber set_servo_damping_sub = node_handle.subscribe("set_servo_damping", 2, set_servo_damping_callback);
    // 创建服务
    ros::ServiceServer query_servo_angle_srv = node_handle.advertiseService("query_servo_angle", query_servo_angle_callback);

    while (ros::ok())
    {
        float angle0, angle1;
        // 通过SDK查询舵机角度
        angle0 = servo0.queryRawAngle();
        angle1 = servo1.queryRawAngle();
        static tf::TransformBroadcaster br; //广播器
        tf::Transform transform;

        //从IMU消息包中获取四元数数据
        tf::Quaternion q;
        // tf2::Quaternion qtn;
        q.setRPY(0, angle1 * PI / 180, -angle0 * PI / 180);
        ROS_INFO("tran_rotation 0.00 %.2f  %.2f  ",  angle1, angle0);
        // q.setX(imu_data->orientation.x);
        // q.setY(imu_data->orientation.y);
        // q.setZ(imu_data->orientation.z);
        // q.setW(imu_data->orientation.w);
        q.normalized(); //归一化

        transform.setRotation(q); //设置旋转部分

        //根据实际所转动的角度计算相机坐标系原点发生发位移变化
        std::vector<float> tran_vec;
        tran_vec = TranDistence(0, angle1, -angle0);
        transform.setOrigin(tf::Vector3(tran_vec[0] + x_offset, tran_vec[1] + y_offset, tran_vec[2] + Height)); //设置平移部分
        ROS_INFO("tran_vec %f  %f  %f  ", tran_vec[0] + x_offset, tran_vec[1] + y_offset, tran_vec[2] + Height);
        //广播出去
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "servo_link", "camera_link"));
        tran_vec.clear();
        ros::spinOnce();
    }
    // 进入循环等待
    // ros::spin();
}