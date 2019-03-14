/*******************************************************************/
/*                      Author: xgt                                */
/*                     Contact: xieguotao1990@126.com              */
/*               First version: 2018-05-21                         */
/*                 Last update: 2018-05-25                         */
/*******************************************************************/

/*
  Description: The header file for C++ class CanDriver.
  The class includes all the necessary vars and funcs for radar information receiving
  based on canet udp socket.
*/
#ifndef N_RADAR_OBJ_CLASS_CANET_UDP_H
#define N_RADAR_OBJ_CLASS_CANET_UDP_H

// Include system lib.
#include <iostream>
#include <sys/socket.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <vector>
#include <fstream>                    // for data storing in txt
#include <boost/thread.hpp>           // mutex in def()
#include "log4cplus/publiclog.h"

// Include personal libs
#include "msg_radar_prep/msg_radar_prep.h"
#include "msg_radar_prep/radar_points.h"
#include "msg_radar_prep/msg_diagn_radar_prep.h"

//Include the test lab for point 3d
#include "sensor_msgs/PointCloud.h"          //add for pointcloud
#include "geometry_msgs/Point32.h"         //add for geometry_point
#include "std_msgs/Header.h"
#include "sensor_msgs/ChannelFloat32.h"

#define BLOCK 0
#define NONBLOCK 1


using namespace std;

//the struct for the radars' parameters based on the vehicle coordination
struct RadarParameters
{
  float x;                          // the parameter of X label/ equipment
  float y;							 // the parameter of Y label
  float z;                          // the parameter of Z label
  //  float pitch;                      // the parameter of pitch
  //  float yaw;                        // the parameter of yaw
  //  float roll;                       // the parameter of roll
  //  char* radar_ip;                     // ???data_sort
  int port_host_rev;
  int port_radar_rev;
  int port_host_send;
  int port_radar_send;
  //  int rate_cycle_index_send;
  //  int rate_cycle_index_rev;
  //  int num;                //the No. of the radar
  string radar_comp;                //the company name of radar
  string radar_type;                //the radar type in the company
  int rate;                         //loop rate;
  float install_x;                 // esr_factor
  float install_a;                  // rsds_factor
  float install_b;                  // rsds_factor
  float install_sin_alfa;           // rsds_factor
  float install_cos_alfa;           // rsds_factor
  //  float alfa;                       // rsds_factor
  //  double threshold;             //loop rate;
};

//the struct for the raw data from radar	
struct RadarRawData
{
  float range;                     //the raw range from radar
  float angle;                     //the raw angle from radar
  float range_rate;                //the raw range_rate from radar
  //  unsigned char data_true;          //the data is true or not(true:1; false:0).
  //  char id_radar;                    //the id for radar
  //  int count=0;
  string install_side;
};

//the struct for radar obj data based on the vehicle coordination
struct RadarObjData
{	

  float x;                     //longitudial information based on the vehicle coordiation
  float y;                     //lateral information based on the vehicle coordiation
  float vx;                  //longitudial velocity from radar based on the vehicle coordination
  float vy;                  //lateral velocity from radar based on the vehicle coordination
  //  unsigned char data_true;
  //  unsigned char dynamic_true;
  string install_side;     //

};


class CanDriver{

public:
  CanDriver();
  ~CanDriver();
  void SetParameter(const char *host_ip, const char *radar_ip, const int prt_host_rev, const int prt_radar_rev, const int prt_host_send, const int prt_radar_send);     //func: set the socket parameters for Canet
  int SocketBuild(void);                                       //func: build the socket for Canet
  int UdpSendCanData(const RadarParameters* radar_para);                                    //func: send data to radar based on Canet
  int UdpRecCanData(const RadarParameters* radar_para);                                     //func: receive data from radar based on Canet
  int DelphiEsrData(const RadarParameters* radar_para, unsigned char rec_buf[]);
  int DelphiRsdsData(const RadarParameters* radar_para, unsigned char rec_buf[]);
  int RadarDataDef(const RadarParameters* radar_para, unsigned char rec_buf[]);                                        //func: get the radar raw data
  int RadarObjNoiseFilter(const RadarParameters* radar_para);                               //func: Process the raw data including noise filtering  and coordination transformation
  int RadarObjCoordTrans(const RadarParameters* radar_para);                                //func: Process the raw data including coordination transformation
  void CloseUdpCanet(const RadarParameters* radar_para);                                    //close the socket

  int sockfd;
  sockaddr_in hostNet;
  sockaddr_in radar_rev; // rev
  sockaddr_in radar_send; // send
  //  sockaddr_in my_addr;

  unsigned char rec_buf[1024]; // receive buffer
  char send_buf[];   // send buffer
  int num_rec_buf;
  int frame_num;
  RadarRawData radar_raw_msg_previous[64];  // perserve the last frame
  RadarRawData radar_raw_msg_hist[1][64];   // radar history raw message

  std::vector<RadarRawData> radar_raw_msg;
  std::vector<RadarObjData> radar_obj_msg;
  std::vector<RadarObjData> radar_obj_msg_vehicle;
  //vector<RadarRawData *> radar_raw_vector;
  int id_radar_msg;
  RadarParameters m_radar_para;    //radar parameters

  msg_radar_prep::msg_radar_prep msg_radar_point3_cloud;        // Message: The pre-processed lidar data. // point to pub in c_radar_prep
  msg_radar_prep::radar_points point3;                        // Message: The pre-processed lidar data. // class:radar_points, object:point3
  msg_radar_prep::msg_diagn_radar_prep radar_diagnose;         //to record the lidar diagnose data;

  geometry_msgs::Point32 points; // this msg make points can measure
  sensor_msgs::PointCloud cloud_output; // msg make cloud_output can rviz

private:

};


#endif  // N_RADAR_OBJ_CLASS_CANET_UDP_H

