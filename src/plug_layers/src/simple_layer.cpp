#include <plug_layers/simple_layer.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/LaserScan.h>
PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)
 
using costmap_2d::LETHAL_OBSTACLE;
 
namespace simple_layer_namespace
{
 
SimpleLayer::SimpleLayer() {}
 
void SimpleLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  depth2laser_sub= nh.subscribe("/pcl_scan", 1, & SimpleLayer::depth2lasercallback,this);
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SimpleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void SimpleLayer::depth2lasercallback(const sensor_msgs::LaserScan::ConstPtr& data)
{
  laser_data = data;
  std::cout<<"points:"<<std::endl;
  // sensor_msgs::LaserScan
  // data
}
 
void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}
 
void SimpleLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;
  if(!laser_data->ranges.size())
    return;
  for(float i=laser_data->angle_min;i<=laser_data->angle_max;i+=laser_data->angle_increment)
  {
    double range=laser_data->ranges[(i- laser_data->angle_min)/laser_data->angle_increment];
    // double range=laser_data->ranges[i];
    if(range>4)
    {
        continue;
    }
    mark_x_ = origin_x + cos(origin_yaw)*range;
    mark_y_ = origin_y + sin(origin_yaw)*range;
    x_points.push_back(mark_x_);
    y_points.push_back(mark_y_);

    *min_x = std::min(*min_x, mark_x_);
    *min_y = std::min(*min_y, mark_y_);
    *max_x = std::max(*max_x, mark_x_);
    *max_y = std::max(*max_y, mark_y_);
  }
  std::cout<<"points:"<<x_points[0]<<std::endl;
  // // mark_x_ = origin_x + cos(origin_yaw);
  // // mark_y_ = origin_y + sin(origin_yaw);
 
  // *min_x = std::min(*min_x, mark_x_);
  // *min_y = std::min(*min_y, mark_y_);
  // *max_x = std::max(*max_x, mark_x_);
  // *max_y = std::max(*max_y, mark_y_);
}
 
void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;
  if(y_points.size()!=x_points.size())
    return;
  if(!y_points.size())
    return;
  unsigned int mx;
  unsigned int my;
  for(int i=0;i<x_points.size();i++)
  {
    if(master_grid.worldToMap(x_points[i], y_points[i], mx, my))
    {
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
    }
  }
  
}
 
} // end namespace
