#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <termio.h>

using namespace std;


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "key_board");
    ros::NodeHandle nh;
    ros::Publisher pub;
    pub = nh.advertise<std_msgs::Int32>("/key_board", 10);
    std_msgs::Int32 key;

    while (ros::ok())
    {
        int in;

        struct termios new_settings;
        struct termios stored_settings;
        //设置终端参数
        tcgetattr(0, &stored_settings);
        new_settings = stored_settings;
        new_settings.c_lflag &= (~ICANON);
        new_settings.c_cc[VTIME] = 0;
        tcgetattr(0, &stored_settings);
        new_settings.c_cc[VMIN] = 1;
        tcsetattr(0, TCSANOW, &new_settings);
        in = getchar();
        if (!ros::ok())
            break;
        tcsetattr(0, TCSANOW, &stored_settings);

        key.data = in;
        cout << "key: " << in << endl;
        cout << "key.data: " << key.data << endl;
        pub.publish(key);
        ros::spinOnce();
    }

    return 0;
}
