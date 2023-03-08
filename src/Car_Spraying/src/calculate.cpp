#include <cmath>
#include <stdio.h>
#include <iostream>

const float Pi = 3.1415926;

int main()
{
    double theta, theta_1, theta_2;
    double r, x, x_1, x_2;
    theta_1 = 10;
    theta_2 = 40;
    x = 0.269;
    x_1 = 0.266;
    x_2 = 0.318;

    theta_1 = (theta_1 / 180.0) * Pi; // 弧度
    theta_2 = (theta_2 / 180.0) * Pi; // 弧度

    // theta = atan((-(x_2 * cos(theta_2) * (cos(theta_1) - 1)) + (x_1 * cos(theta_1) * (cos(theta_2) - 1)) + (x * (cos(theta_1) - 1)) - (x * (cos(theta_2) - 1))) /
    //              ((x_2 * cos(theta_2) * sin(theta_1)) - (x_1 * cos(theta_1) * sin(theta_2)) - (x * sin(theta_1)) + (x * sin(theta_2)))); // 弧度
    theta = atan(((x - x_1 * cos(theta_1)) * (cos(theta_2) - 1) - (x - x_2 * cos(theta_2)) * (cos(theta_1) - 1)) /
                 (sin(theta_1) * (x - x_2 * cos(theta_2)) - sin(theta_2) * (x - x_1 * cos(theta_1)))); // 弧度
    r = (-x_1 * cos(theta_1) + x) / (cos(theta - theta_1) - cos(theta));

    std::cout << "theta = " << theta << std::endl;
    std::cout << "r = " << r << std::endl;

    r = (-x_2 * cos(theta_2) + x) / (cos(theta - theta_2) - cos(theta));
    std::cout << "r = " << r << std::endl;

    return 0;
}
