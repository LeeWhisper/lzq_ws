#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "track_avoid/Depth.h"
#include <geometry_msgs/Twist.h>

#include "iostream"
using namespace std;

class move
{
private:
    /* data */
    ros::NodeHandle nh;
    ros::Subscriber sub_person, sub_avoid_flag;
    ros::Publisher pub_move;
    bool avoid_flag;

    void move_callback(const track_avoid::Depth depth_person);
    void avoid_flag_callback(const std_msgs::Bool flag);
public:
    
    move();
};

move::move()
{  
    sub_person = nh.subscribe("/track_person", 10, &move::move_callback, this);
    sub_avoid_flag = nh.subscribe("/avoid_flag", 10, &move::avoid_flag_callback, this);
    pub_move = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
}

void move::avoid_flag_callback(const std_msgs::Bool flag)
{
    if (flag.data)
        avoid_flag = true;
    else
        avoid_flag = false;
    
}
void move::move_callback(const track_avoid::Depth depth_person)
{
    if (avoid_flag == false)
    {
       	geometry_msgs::Twist mv;
        if (depth_person.locate < 280)
        {
            cout << "人物位于左边" << endl;
            mv.angular.z = 0.1;//左转
        }
        if (depth_person.locate >= 280 && depth_person.locate <= 360)
        {
            cout << "人物位于中间" << endl;
            mv.angular.z = 0;
        }
        if (depth_person.locate > 360)
        {
            cout << "人物位于右边" << endl;
            mv.angular.z = -0.1;//右转
        }

        if (depth_person.depth > 150)
        {
            mv.linear.x = 0.1;
        }
        else
        {
            mv.linear.x = 0;
            cout << "距离小于1.5m" << endl;
        }

        pub_move.publish(mv);
    }
    else
    {
        cout << "执行避障";
    }
}

int main(int argc, char  *argv[])
{
    ros::init(argc, argv, "move_01");
    class move m;

    ros::spin();
    return 0;
}
