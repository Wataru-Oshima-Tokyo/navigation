#include <go_back_recovery/go_back_recovery.h>
#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <algorithm>
#include <pluginlib/class_list_macros.h>

//register this planner as a RecoveryBehavior plugin
//PLUGINLIB_EXPORT_CLASS(go_back_recovery, GobackRecovery, go_back_recovery::GoBackRecovery, nav_core::RecoveryBehavior)
PLUGINLIB_EXPORT_CLASS(go_back_recovery::GoBackRecovery, nav_core::RecoveryBehavior)

namespace go_back_recovery {

 GoBackRecovery::GoBackRecovery(): global_costmap_(NULL), local_costmap_(NULL), tf_(NULL), initialized_(false), world_model_(NULL) {}

  void GoBackRecovery::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
  if(!initialized_){
   name_=name;
   tf_ = tf;
   global_costmap_=global_costmap;
   local_costmap_=local_costmap; 
   world_model_=new base_local_planner::CostmapModel(*local_costmap_->getCostmap());
   initialized_=true;
   }else {
    ROS_ERROR("You should not call initialize twice on this project, doing nothing");
   }
  }

 GoBackRecovery::~GoBackRecovery(){
   delete world_model_;
 }

 double GoBackRecovery::null_check(double target){
  if(!(target >0)){
   target=(double)RANGE_MAX;
  }
  return target;
 }

 void GoBackRecovery::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  double center_number=(-msg->angle_min)/msg->angle_increment;
  double center=msg->ranges[center_number+180];
  double left=msg->ranges[center_number+128];
  double right=msg->ranges[center_number-128];

  center=null_check(center);
  left=null_check(left);
  right=null_check(right);
 
  ROS_INFO("center:[%If], left:[%If], right:[%If]", center, left, right);
  ROS_INFO("center number: [%If]", (-msg->angle_min)/msg->angle_increment);

  if(center < 0.5){
   ROS_WARN("center warning!!");
   cmd_vel.linear.x = 0.0;
   cmd_vel.linear.y = 0.0;
   cmd_vel.angular.z = 0.5;
  }
  if(left < 0.4){
   ROS_WARN("left warning!!");
   cmd_vel.linear.x = 0.0;
   cmd_vel.linear.y = 0.0;
   cmd_vel.angular.z = -0.5;
  }
  if(right < 0.4){
   ROS_WARN("right warning!!");
   cmd_vel.linear.x = 0.0;
   cmd_vel.linear.y = 0.0;
   cmd_vel.angular.z = 0.5;
  }
  if(center >= 0.5 && left >=0.4 && right >=0.4){
   cmd_vel.linear.x = -0.2;
   cmd_vel.linear.y = 0.0;
   cmd_vel.angular.z = 0.0;
  }

  vel_pub.publish(cmd_vel);
  ROS_INFO("sub_n: %d", sub_n);
  sub_n++;
  if(sub_n >100) {
      scan_sub.shutdown();
      sub_flag=0;
      return;
  }
 }

 void GoBackRecovery::runBehavior(){
   if(!initialized_){
     ROS_ERROR("This object must be initialized before runBehavior is called");
     return;
   }

   ROS_WARN("Go back Recovery behavior started");
   sub_n =0;
   sub_flag=1;
   scan_sub=n.subscribe("scan", 10, &GoBackRecovery::scanCallback, this);
   vel_pub=n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
   while(n.ok()){
      if(sub_flag==0) {
        return;
      }
   }
 }
};


