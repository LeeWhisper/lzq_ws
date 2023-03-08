#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <cmath>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "points_and_lines");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("/home/lzq/lzq_ws/pointcloud/3.1/sort.pcd", *cloud);
    float min_z, max_z, min_y, max_y, min_x;
    for (int i = 0; i < cloud->size(); i++)
    {
        if (min_z > cloud->points[i].z)
            min_z = cloud->points[i].z;
        if (max_z < cloud->points[i].z)
            max_z = cloud->points[i].z;
        if (min_y > cloud->points[i].y)
            min_y = cloud->points[i].y;
        if (max_y < cloud->points[i].y)
            max_y = cloud->points[i].y;
        if (min_x > cloud->points[i].x)
            min_x = cloud->points[i].x;
    }

    std::cout << "min_z: " << min_z << "max_z: " << max_z << "min_y: " << min_y << "max_y: " << max_y << std::endl;

    std::cout << "min_x: " << min_x << std::endl;
    ros::Rate r(30);

    float f = 0.0;
    while (ros::ok())
    {
        visualization_msgs::Marker points, line_strip, line_list;
        points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "my_frame";
        points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
        points.ns = line_strip.ns = line_list.ns = "points_and_lines";
        points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

        points.id = 0;
        line_strip.id = 1;
        line_list.id = 2;

        points.type = visualization_msgs::Marker::POINTS;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_list.type = visualization_msgs::Marker::LINE_LIST;

        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.01;
        points.scale.y = 0.01;
        points.scale.z = 0.01;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.005;
        line_strip.scale.y = 0.005;
        line_strip.scale.z = 0.005;

        line_list.scale.x = 0.005;

        // Points are green
        points.color.g = 1.0f;
        points.color.a = 1.0;

        // Line strip is blue
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        // Line list is red
        line_list.color.r = 1.0;
        line_list.color.a = 1.0;

        // Create the vertices for the points and lines
        // for (uint32_t i = 0; i < 100; ++i)
        // {
        //     float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
        //     float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

        //     geometry_msgs::Point p;
        //     p.x = (int32_t)i - 50;
        //     p.y = y;
        //     p.z = z;

        //     points.points.push_back(p);
        //     line_strip.points.push_back(p);

        //     // The line list needs two points for each line
        //     line_list.points.push_back(p);
        //     p.z += 1.0;
        //     line_list.points.push_back(p);
        // }
        float min_x = 10;
        int count = 0;
        for (int i = 0; i < cloud->points.size(); ++i)
        {
            geometry_msgs::Point p;
            p.x = cloud->points[i].x;
            p.y = cloud->points[i].y;
            p.z = cloud->points[i].z;
            points.points.push_back(p);
            line_strip.points.push_back(p);
            // line_list.points.push_back(p);
            // p.x -= 0.01;
            // line_list.points.push_back(p);
            if (min_x > cloud->points[i].x)
                min_x = cloud->points[i].x;
            count++;
        }
        // std::cout << "count:" << count << std::endl;
        marker_pub.publish(points);
        marker_pub.publish(line_strip);
        // marker_pub.publish(line_list);
        r.sleep();
        // f += 0.04;
    }
}