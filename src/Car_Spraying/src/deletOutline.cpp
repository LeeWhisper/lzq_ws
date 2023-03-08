#include <iostream>
#include <algorithm>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>

using namespace std;

struct BGR // 定义BGR结构体
{
    uchar b;
    uchar g;
    uchar r;
};
struct HSV // 定义HSV结构体
{
    int h;
    double s;
    double v;
};

bool IsEquals(double val1, double val2)
{
    return fabs(val1 - val2) < 0.001;
}

// 将RGB格式转换为HSV格式
void BGR2HSV(BGR &bgr, HSV &hsv)
{
    double b, g, r;
    double h, s, v;
    double min, max;
    double delta;

    b = bgr.b / 255.0;
    g = bgr.g / 255.0;
    r = bgr.r / 255.0;

    if (r > g)
    {
        max = MAX(r, b);
        min = MIN(g, b);
    }
    else
    {
        max = MAX(g, b);
        min = MIN(r, b);
    }

    v = max;
    delta = max - min;

    if (IsEquals(max, 0))
        s = 0.0;
    else
        s = delta / max;

    if (max == min)
        h = 0.0;
    else
    {
        if (IsEquals(r, max) && g >= b)
        {
            h = 60 * (g - b) / delta + 0;
        }
        else if (IsEquals(r, max) && g < b)
        {
            h = 60 * (g - b) / delta + 360;
        }
        else if (IsEquals(g, max))
        {
            h = 60 * (b - r) / delta + 120;
        }
        else if (IsEquals(b, max))
        {
            h = 60 * (r - g) / delta + 240;
        }
    }

    hsv.h = (int)(h + 0.5);
    hsv.h = (hsv.h > 359) ? (hsv.h - 360) : hsv.h;
    hsv.h = (hsv.h < 0) ? (hsv.h + 360) : hsv.h;
    hsv.s = s;
    hsv.v = v;
    hsv.h = hsv.h / 2;
    hsv.s = hsv.s * 255;
    hsv.v = hsv.v * 255;
}

bool color(pcl::PointXYZRGB point)
{
    bool flag = 0;
    int R, G, B;
    int min_R, max_R, min_G, max_G, min_B, max_B;
    int min_H, max_H, min_S, max_S, min_V, max_V;

    R = (int)point.r;
    G = (int)point.g;
    B = (int)point.b;

    BGR bgr;
    bgr.r = (int)point.r;
    bgr.g = (int)point.g;
    bgr.b = (int)point.b;
    HSV hsv;
    BGR2HSV(bgr, hsv); // bgr转hsvI
    // min_R = 100;
    // max_R = 175;
    // min_G = 50;
    // max_G = 110;
    // min_B = 0;
    // max_B = 100;

    // yellow_RGB
    min_R = 100;
    max_R = 255;
    min_G = 140;
    max_G = 255;
    min_B = 0;
    max_B = 175;

    // red_RGB
    // min_R = 140;
    // max_R = 255;
    // min_G = 10;
    // max_G = 150;
    // min_B = 0;
    // max_B = 200;

    if (R >= min_R && R <= max_R && G >= min_G && G <= max_G && B >= min_B && B <= max_B)
    {
        flag = 1;
    }

    // red_HSV
    // min_H = 150;
    // max_H = 10;
    // min_S = 43;
    // max_S = 255;
    // min_V = 46;
    // max_V = 255;
    // if (min_H < max_H)
    // {
    //     if (hsv.h <= max_H && hsv.h >= min_H && hsv.s >= min_S && hsv.s <= max_S && hsv.v >= min_V && hsv.v <= max_V)
    //         flag = 1;
    // }
    // else if (min_H > max_H)
    // {
    //     if ((hsv.h <= max_H || hsv.h >= min_H) && hsv.s >= min_S && hsv.s <= max_S && hsv.v >= min_V && hsv.v <= max_V)
    //         flag = 1;
    // }

    return flag;
}

int main(int argc, char const *argv[])
{
    //----------------------读取点云---------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/lzq/lzq_ws/pointcloud/3.1/hole_fix.pcd", *cloud) == -1)
    {
        PCL_ERROR("Cloudn't read file!");
        return -1;
    }

    vector<int> total_index;
    for (size_t i = 0; i < cloud->size(); i++)
    {
        if (color(cloud->points[i]))
        {
            total_index.push_back(i);
        }
        // std::cout << "point" << i << ":" << cloud->points[i].x;
        // std::cout << "," << cloud->points[i].y;
        // std::cout << "," << cloud->points[i].z;
        // std::cout << "," << int(cloud->points[i].r);
        // std::cout << "," << int(cloud->points[i].g);
        // std::cout << "," << int(cloud->points[i].b) << std::endl;
    }
    sort(total_index.begin(), total_index.end()); // 将索引进行排序
    //-------------------根据索引删除重复的点-------------------
    pcl::PointIndices::Ptr outliners(new pcl::PointIndices());
    outliners->indices.resize(total_index.size());
    for (size_t i = 0; i < total_index.size(); i++)
    {
        outliners->indices[i] = total_index[i];
    }
    cout << "点云删除完毕！！！" << endl;
    //-------------------提取删除重复点之后的点云--------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(outliners);
    extract.setNegative(true); // 设置为true则表示保存索引之外的点
    extract.filter(*cloud_filtered);
    cout << "原始点云中点的个数为：" << cloud->points.size() << endl;
    cout << "删除的点的个数为:" << total_index.size() << endl;
    cout << "去掉之后点的个数为:" << cloud_filtered->points.size() << endl;
    //-------------------------保存点云-------------------------
    pcl::PCDWriter writer;
    writer.write("/home/lzq/lzq_ws/pointcloud/3.1/Hf_De.pcd", *cloud_filtered, true);
    //-------------------------可视化-------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud_filtered, 0, 255, 0); // green

    viewer->addPointCloud<pcl::PointXYZRGB>(cloud_filtered /* , single_color */, "sample cloud");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return 0;
}
