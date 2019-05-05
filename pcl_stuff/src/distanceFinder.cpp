#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/UInt8.h>
#include <math.h>
#include <iostream>

using namespace std;

//global variables
sensor_msgs::PointCloud2 input_pointcloud;
sensor_msgs::PointCloud newCloud;
std_msgs::Int32 distanceFound;
sensor_msgs::PointCloud OrbsOfInterest;
geometry_msgs::PoseStamped CurrPose;
std_msgs::Int32MultiArray ColorXY;
bool isCentered;

//callback functions for each subscriptions that puts the message into the respective global variables
void PointCloudCallBack(const sensor_msgs::PointCloud2 &msg) {
  sensor_msgs::convertPointCloud2ToPointCloud(msg, newCloud);
}
void PoseCallBack(const geometry_msgs::PoseStamped msg) {
  CurrPose = msg;
}
void ObjectCenteredCallBack(const std_msgs::Bool msg) {
  isCentered = msg.data;
}


//main
int main(int argc, char **argv) {
  ros::init(argc, argv, "distanceFinder");
  ros::NodeHandle n;
  //all of the topics this node is publishing
  ros::Publisher distancePub = n.advertise<std_msgs::Int32>("bebop/ObjectDistance",10);
  ros::Publisher distancePointsPub = n.advertise<sensor_msgs::PointCloud>("bebop/foundOrbsAroundObject", 1);
  ros::Publisher startFlightPlanPub = n.advertise<std_msgs::String>("bebop/autoflight/start", 1);
  ros::Publisher doFlipPub = n.advertise<std_msgs::UInt8>("bebop/flip", 1);
  //All of the topics this is subscribed to
  //ros::Subscriber PointCloudSub = n.subscribe("orb_slam2_mono/map_points", 1, PointCloudCallBack);
  //ros::Subscriber PoseSub = n.subscribe("orb_slam2_mono/pose", 1, PoseCallBack);
  ros::Subscriber objectCenter_sub = n.subscribe("bebop/ObjectCentered", 1, ObjectCenteredCallBack);
  //this node will run at .5 Hz
  ros::Rate loop_rate(.2);
  /*
  this is the main loop. we can have the drone let us know when it has centered on an object; once that happens, we then pull the pose from orb_slam2_mono/pose and use
  the orientation to figure out where the object is relative to orbs that we have already collected, and put probable orbs into OrbsOfInterest PointCloud which
  we then compute/find the distance the drone is from the most dense collection of orbs. We can do this several times and average it to get a more accurate
  reading of the distance.
  */

  while (ros::ok()) {
    if (isCentered == true) {
      doFlipPub.publish(1);
      loop_rate.sleep();
      startFlightPlanPub.publish("flightPlan.mavlink");
    }
    ros::spinOnce();













    /*
    bool clusterFound = false;
    while(isCentered == true) {
      clusterFound = false;

      this for loop is where all of the calculations for determining deired orbs will go

      geometry_msgs::Point searchPoint;
      int count = 1;
      while(clusterFound == false) {
        sensor_msgs::PointCloud tempCloud;
        searchPoint.x = CurrPose.pose.position.x + (CurrPose.pose.orientation.x) * count;
        searchPoint.y = CurrPose.pose.position.y + (CurrPose.pose.orientation.y) * count;
        searchPoint.z = CurrPose.pose.position.z + (CurrPose.pose.orientation.z) * count;
        int orbCount = 0;
        for(geometry_msgs::Point32 p : newCloud.points) {
          if(sqrt(pow((p.x - searchPoint.x),2) + pow((p.y - searchPoint.y),2) + pow((p.z - searchPoint.z),2)) < 15) {
            tempCloud.points.push_back(p);
            orbCount++;
            if(orbCount == 1) { //once we get to desired amount
              clusterFound = true;
              OrbsOfInterest = tempCloud;
              break;
            }
          }
        }
        //end of for loop
        if (count > 50) {
          cout << "breaking out of while loop, count = " << count << endl;
          break;
        }
        count++;
      }
      if (clusterFound == true) {
        std_msgs::Int32MultiArray allDistances;
        //this for loop is acting like a while loop
        for(int i = 0; i < OrbsOfInterest.points.size(); i++) {
          geometry_msgs::Point32 p = OrbsOfInterest.points[i];
          allDistances.data.push_back(sqrt(pow((p.x - CurrPose.pose.position.x),2) + pow((p.y - CurrPose.pose.position.y),2) + pow((p.z - CurrPose.pose.position.z),2)));
        }
        std_msgs::Int32 total;
        total.data = 0;
        for(int i = 0; i < allDistances.data.size(); i++) {
          std_msgs::Int32 d;
          d.data = allDistances.data[i];
          total.data = total.data + d.data;
        }
        int all = allDistances.data.size();
        std_msgs::Int32 newAll;
        newAll.data = all;
        distanceFound.data = total.data / newAll.data;
        cout << "distanceFound: " << distanceFound.data << endl;
        loop_rate.sleep();
        distancePub.publish(distanceFound);
        distancePointsPub.publish(OrbsOfInterest);
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
    //end of while loop
    //IF WHILE LOOP IS SKIPPED, IT SKIPS TO HERE
    //*********************************************************************************************************
    /*
    here we will get objectDistance by running through OrbsOfInterest and getting the average distance of each orb from
    the current pose and returning that distance to be published


    ros::spinOnce();
    loop_rate.sleep();
  }
  */
  }
  return 0;
}
