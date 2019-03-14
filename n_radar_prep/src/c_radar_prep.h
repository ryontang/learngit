/*******************************************************************/
/*                      Author: xgt                                */
/*                     Contact: xieguotao1990@126.com              */
/*               First version: 2018-05-10                         */
/*                 Last update: 2018-10-22                         */
/*******************************************************************/

/*
  Description: The header file for C++ class CRadarObj.
  The class includes all the necessary vars and funcs for radar based
  estimating.
*/


// Syntax: <package name>_<type>_<file name>_H
#ifndef N_RADAR_OBJ_CLASS_C_RADAR_OBJ_H
#define N_RADAR_OBJ_CLASS_C_RADAR_OBJ_H


// Include system lib.
#include "ros/ros.h"
#include <string>
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"

#include <sensor_msgs/PointCloud.h>
#include <ros/time.h>
#include <ros/timer.h>

//boost library
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/variant.hpp>

// Include personal lib.
#include "msg_radar_prep/radar_points.h"
#include "msg_radar_prep/msg_diagn_radar_prep.h"
#include "msg_radar_prep/msg_radar_prep.h"
#include "msg_v_state/msg_v_state.h"
#include "operator_ini.h"
#include "my_typedef.h"

#include "c_can_driver.h"                   //the class for canetUdp

#define NUM_RADAR 2                        //the total number of radars on the platform
//#define TIME 4                           //the total time of the object trajectory



namespace radar
{
using namespace std;

class CRadarPrep
{
public:
  CRadarPrep(){} // fault

  void ClassInit(string cfg_file_path);                              // init not just private-Variable also cannet-radar-Variable, so use Classinit() not constructor
  //  void SendVehicleData();                        // Send Vehicle data to the radar.
  void PubDiagnose();
  void AcqVehicleData(const msg_v_state::msg_v_stateConstPtr &msg);   // Acquire vehicle data from the ethernet.
  void RadarDataPrep(); // Radar raw data pre-process including data_parse, noise filter, and Coordinate transf
  void PubRadarPrep();
  //  void PubPrepData();                                             // Pub prep data through topic.
  ~CRadarPrep();
  static int radar_nums;

  std::vector<CanDriver*> can_driver;                               // !!! Canet class for the radar,you need use class(candriver) in class(cradarprep)!!!

private:
  ros::NodeHandle n;                                      // Main access to ROS.
  ros::NodeHandle n_test;
  ros::Publisher pub_radar_prep;                                // ROS pub obj for prep data;
  ros::Publisher pub_diagnose_prep;
  ros::Publisher pub_radar_esr;
  ros::Publisher pub_radar_rsds;
  ros::Publisher pub_test_point;
  ros::Subscriber sub_msg_v_state;                                   // ROS Subscriber vehicle.

  //msg_radar_prep::radar_points msg_my_radar_raw;                                  // Store the raw data from the ethernet.
  //msg_obj_fuse::obj msg_my_radar_prep;                                // Message: The pre-processed radar data.
  //msg_radar_prep::msg_radar_prep msg_radar_prep;                   // Message: The pre-processed radar data.
  msg_radar_prep::msg_radar_prep msg_radar_prep_data;                   // Message: The pre-processed radar data.
  msg_radar_prep::msg_diagn_radar_prep radar_prep_diagnose;
  msg_v_state::msg_v_state msg_v_state_pred;                        // msg: Results from vehicle.
  //msg_radar_prep::msg_radar_prep radar_prep; //???

  sensor_msgs::PointCloud cloud_output; // two cloud_outpub used for dependently-whole pub
  std::vector<sensor_msgs::PointCloud> each_radar_cloud;
  //RadarParameters radar_para[NUM_RADAR];

  int32 tmp_count;                                        // Test conter.

};  // class CRadarObj


}  // namespace xgt


#endif  // N_RADAR_OBJ_CLASS_C_RADAR_OBJ_H
