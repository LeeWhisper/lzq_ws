#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "ctime"
#include "opencv2/opencv.hpp"
#include "track_avoid/Depth.h"

using namespace std;
using namespace cv;

class avoid
{
private:
    /* data */
    ros::NodeHandle nh;
    ros::Subscriber sub_laser, sub_get_flag, sub_person;
    ros::Publisher pub_avoid_move, pub_avoid_flag;
    vector<float> ranges;
    int right_count, left_count;
    int turn_right_count = 0, turn_left_count = 0;
    bool get_flag, depth_flag;
    std_msgs::Bool avoid_flag;
    track_avoid::Depth depth_person;

    void avoid_callback(sensor_msgs::LaserScan scan_msg);
    void get_flag_callback(std_msgs::Bool flag);
    void turn_left(geometry_msgs::Twist avoid_move);
    void turn_right(geometry_msgs::Twist avoid_move);
    void move_callback(track_avoid::Depth depth);

    void goback();
public:
    avoid(/* args */);
};

avoid::avoid(/* args */)
{
    avoid_flag.data = false;
    sub_laser = nh.subscribe("/scan", 1000, &avoid::avoid_callback, this);
    sub_get_flag = nh.subscribe("/get_flag", 1,&avoid::get_flag_callback, this);
    sub_person = nh.subscribe("/track_person", 10, &avoid::move_callback, this);

    pub_avoid_move = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    pub_avoid_flag = nh.advertise<std_msgs::Bool>("/avoid_flag", 10);
}

void avoid::get_flag_callback(std_msgs::Bool flag)
{
    if (flag.data)
        get_flag = true;
    else
        get_flag = false;
    cout << "get_flag: " << get_flag << endl;
}

void avoid::move_callback(track_avoid::Depth depth)
{
    depth_person = depth;
}

void avoid::turn_left(geometry_msgs::Twist avoid_move)
{
    avoid_move.angular.z = 0.3;
    avoid_move.linear.x = 0.07;
    
}

void avoid::turn_right(geometry_msgs::Twist avoid_move)
{
    avoid_move.angular.z = -0.3;
    avoid_move.linear.x = 0.07;
}

void avoid::avoid_callback(sensor_msgs::LaserScan scan_msg)
{
    right_count = 0;
    left_count = 0;
    int count;
    bool count_flag = false, flag = false;
    ranges = scan_msg.ranges;
    geometry_msgs::Twist avoid_move;

    for (int i = 360; i <= 1080; i++)
    {
        if (ranges[i] == 0)
        {
            ranges[i] = 0.5;
        }
    }

    //判断右侧障碍物数量
    for (int i = 360; i < 720; i++)
    {
        //连续20个点小于0.4m才会记录为障碍物
        if (ranges[i] <= 0.4)
        {
            count_flag = true;
            count = i;
            for (int i = 0; i < 20; i ++)
            {
                if (ranges[count + i] <= 0.4)
                    flag = true;
                else
                    flag = false;
                count_flag = flag && count_flag;
            }
        }
        if (count_flag)
        {
            right_count += 1;
        }
    }

    //判断左侧障碍物
    for (int i = 720; i < 1080; i++)
    {
        if (ranges[i] <= 0.4)
        {
            count_flag = true;
            count = i;
            for (int i = 0; i < 20; i ++)
            {
                if (ranges[count + i] <= 0.4)
                    flag = true;
                else
                    flag = false;
                count_flag = flag && count_flag;
            }
        }
        if (count_flag)
        {
            left_count += 1;
        }
    }

    cout << "right_count: " << right_count << endl;
    cout << "left_count: " << left_count << endl;
    if (right_count > left_count)//左转
    {
        avoid_flag.data = true;
        // avoid::turn_left(avoid_move);
        avoid_move.angular.z = 0.3;
        avoid_move.linear.x = 0.07;
        turn_left_count += 1;
        cout << "turn_left" << endl;
        pub_avoid_move.publish(avoid_move);

    }

    if (left_count > right_count)//右转
    {
        avoid_flag.data = true;
        // avoid::turn_right(avoid_move);
        avoid_move.angular.z = -0.3;
        avoid_move.linear.x = 0.07;
        turn_right_count += 1;
        cout << "turn_right" << endl;
        pub_avoid_move.publish(avoid_move);
    }

    // pub_avoid_move.publish(avoid_move);

    if (avoid_flag.data == false )
    {
        for (turn_left_count; turn_left_count > 0; turn_left_count --)
        {
            // turn_right(avoid_move);
            avoid_move.angular.z = -0.35;
            cout << "left_back" << endl;
            cout << "turn_left_count" << turn_left_count << endl;
            pub_avoid_move.publish(avoid_move);

        }
        for (turn_right_count; turn_right_count > 0; turn_right_count --)
        {
            // turn_left(avoid_move);
            avoid_move.angular.z = 0.35;
            cout << "right_back" << endl;
            cout << "turn_right_count" << turn_right_count << endl;
            pub_avoid_move.publish(avoid_move);

        }
    }

    if (depth_person.depth < 150)
        depth_flag = false;
    else
        depth_flag = true;

    if (get_flag == false && avoid_flag.data == false)
    {
        avoid_move.angular.z = -0.1;
        cout << "寻找目标" << endl;
        pub_avoid_move.publish(avoid_move);
    }

    // avoid_move.linear.x = 0.05;
    // pub_avoid_move.publish(avoid_move);
    pub_avoid_flag.publish(avoid_flag);

    avoid_flag.data = false;

    // waitKey(1);
}

void avoid::goback()
{

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "aviod_01");
    class avoid av;

    ros::spin();
    return 0;
}
