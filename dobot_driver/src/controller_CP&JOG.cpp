#include <stdint.h>
#include <queue>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "dobot_api/DobotDll.h"

geometry_msgs::Twist desired__;
ros::Time timeout__;
bool r_moving__ = false;

void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  desired__ = *msg;
  timeout__ = ros::Time::now() + ros::Duration(0.5);
}

#include "dobot_msgs/GetPose.h"

bool GetPoseService(dobot_msgs::GetPose::Request &req, dobot_msgs::GetPose::Response &res)
{
    Pose pose;

    res.result = GetPose(&pose);
    if (res.result == DobotCommunicate_NoError) {
        res.x = pose.x;
        res.y = pose.y;
        res.z = pose.z;
        res.r = pose.r;
        for (int i = 0; i < 4; i++) {
            res.jointAngle.push_back(pose.jointAngle[i]);
        }
    }

    return true;
}

void InitPoseServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/GetPose", GetPoseService);
    serverVec.push_back(server);
}

uint64_t enqueue(const geometry_msgs::Twist& msg)
{
  float vel = sqrt(pow(msg.linear.x, 2) + pow(msg.linear.y, 2) + pow(msg.linear.z, 2));
  // CP
  CPParams params;
  params.planAcc = 200;
  params.juncitionVel = 1000*vel;
  params.acc = 100;
  params.realTimeTrack = 1;
  params.period = 1;
  
  CPCmd cmd;
  cmd.cpMode = CPRelativeMode;
  if (vel > 0)
  {
    // Relative position after 10ms at this speed
    cmd.x = 10*msg.linear.x;
    cmd.y = 10*msg.linear.y;
    cmd.z = 10*msg.linear.z;
  }
  else
    cmd.x = cmd.y = cmd.z = 0;
  cmd.velocity = vel;

  // JOG

  JOGCoordinateParams jogParams;
  float vel_list[4] = {0, 0, 0, 100};
  float acc_list[4] = {0, 0, 0, 200};
  for (auto i = 0; i < 4; i++)
  {
    jogParams.velocity[i] = vel_list[i];
    jogParams.acceleration[i] = acc_list[i];
  }
  

  JOGCmd jogCmd;
  jogCmd.isJoint = false;
  if (msg.angular.z > 0){
    jogCmd.cmd = 7;
  }else if (msg.angular.z < 0){
    jogCmd.cmd = 8;
  }else{
    jogCmd.cmd = 0;
  }
  
  //        | xyz move | stop
  // r move |   x          o         
  // r stop |   o          o       
    
  uint64_t index;
  if (vel > 0)
  {
    if (r_moving__)
    {
      SetJOGCmd(&jogCmd, false, &index);
      r_moving__ = false;
    }
    SetCPParams(&params, false, &index);
    SetCPCmd(&cmd, true, &index);
    ROS_INFO("CP: %f, %f, %f, %f", cmd.x, cmd.y, cmd.z, cmd.velocity);
  }
  else
  {
    SetJOGCoordinateParams(&jogParams, false, &index);
    SetJOGCmd(&jogCmd, true, &index);
    r_moving__ = true;
  }
  return index;
}

int main(int argc, char **argv)
{
  ROS_INFO("updated");
  ros::init(argc, argv, "dobot_controller");
  ros::NodeHandle nh;
  
  std::string device;
  nh.param<std::string>("device", device, "/dev/ttyUSB0");
  
  int queue_depth;
  nh.param<int>("queue_depth", queue_depth, 2);
  
  // Connect Dobot before starting the service
  int result = ConnectDobot(device.c_str(), 115200, 0, 0);
  switch (result)
  {
    case DobotConnect_NoError:
      break;
    case DobotConnect_NotFound:
      ROS_ERROR("Dobot not found");
      return 1;
    case DobotConnect_Occupied:
      ROS_ERROR_STREAM("Invalid port name " << device << " or Dobot is occupied by another application");
      return 2;
    default:
      break;
  }
  
  ros::Subscriber sub = nh.subscribe("cmd_vel", 1000, velocityCallback);
  std::vector<ros::ServiceServer> serverVec;
  InitPoseServices(nh, serverVec);
  
  SetQueuedCmdClear();
  SetQueuedCmdStartExec();
  
  uint64_t index;
  GetQueuedCmdCurrentIndex(&index);

  ros::Rate rate(1000);
  while (ros::ok())
  {
    if (ros::Time::now() < timeout__)
    {
      uint64_t current_index;
      GetQueuedCmdCurrentIndex(&current_index);
      while (index-current_index < 2*queue_depth)
        // ROS_INFO("index: %ld, current index: %ld", index, current_index);
        index = enqueue(desired__);
    }
    
    ros::spinOnce();
    rate.sleep();
  }
  
  SetQueuedCmdForceStopExec();
  
  return 0;
}
