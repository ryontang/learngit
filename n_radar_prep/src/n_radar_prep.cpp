/*******************************************************************/
/*                      Author: xgt                                */
/*                     Contact: xieguotao1990@126.com              */
/*               First version: 2018-05-10                         */
/*                 Last update: 2018-05-11                         */
/*******************************************************************/

/*
  Description: The source code for node n_radar_obj.
  early vision。can run sucessiful!
*/

// Include system lib.
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pthread.h>                      // Thread func header.
#include <unistd.h>                       // For system time count.

// Include personal lib.
#include "msg_radar_prep/msg_radar_prep.h"
#include "msg_radar_prep/radar_points.h"
#include "msg_radar_prep/msg_diagn_radar_prep.h"
#include "c_radar_prep.h"
#include "my_typedef.h"
#include <boost/thread.hpp>
#include "log4cplus/publiclog.h"

Logger logger_public = Logger::getInstance( LOG4CPLUS_TEXT( "log" ) );
Logger root = Logger::getRoot( );

//func prototype
void* ThreadRadarRecv(void *arg, int flag);   //to received all lidar data thread
void* ThreadRadarSend(void *arg, int flag);   //to received all lidar data thread
void *ThreadPubRadarPrep(void* arg);              //to pub prep lidar data
void *ThreadPubDiagnose(void * arg);
//void* ThreadAcqVehicleData(void* arg);    // Acquire vehicle data from the ethernet.

int main(int argc, char **argv)
{
  ros::init(argc, argv, "n_radar_obj");   // ROS init.
  PropertyConfigurator::doConfigure( LOG4CPLUS_TEXT( LOG4CXX_RADAR_FILE_PATH_LOG ) ); // to assign the log's configure file
  radar::CRadarPrep* radar_prep = new radar::CRadarPrep();             // Class def.
  radar_prep->ClassInit("/home/aaa/junfeng/src/nodes/n_radar_prep/cfg/n_radar_prep.ini");                // Class init.

  vector<boost::thread *> recv_radar_thread; // store all receive radar data thread???how is data perserve
  vector<boost::thread *> send_radar_thread; // store all receive radar data thread

  for(int i = 0; i < radar::CRadarPrep::radar_nums; ++i)
  {
    recv_radar_thread.emplace_back(new boost::thread(boost::bind(ThreadRadarRecv ,radar_prep, i)));
    send_radar_thread.emplace_back(new boost::thread(boost::bind(ThreadRadarSend ,radar_prep, i)));
  }

  boost::thread* radar_prep_diagn  = new boost::thread(boost::bind(ThreadPubDiagnose,radar_prep));
  // ???why no boost::thread* radar_prep
  boost::thread* radar_pub_prep  = new boost::thread(boost::bind(ThreadPubRadarPrep,radar_prep)); // preparetions for pub_radar_prep

  for(int i = 0; i < recv_radar_thread.size(); ++i )
  {
    recv_radar_thread[i]->interrupt();
    recv_radar_thread[i]->join();
    send_radar_thread[i]->interrupt();
    send_radar_thread[i]->join();
  }
  radar_prep_diagn->interrupt();
  radar_prep_diagn->join();
  radar_pub_prep->interrupt(); // correspond 132 line(make preparation), start thread right now
  radar_pub_prep->join(); // make the thread end then main can be end

  delete radar_prep;
  return 0;
}

//func difinition
void* ThreadRadarRecv(void* arg, int flag)
{
  ROS_ERROR("1ThreadRadarRecv");
  radar::CRadarPrep* radar_prep = (radar::CRadarPrep*)arg;
  int tmp_count = 0;
  while (ros::ok())
  {
    radar_prep->can_driver[flag]->UdpRecCanData(&radar_prep->can_driver[flag]->m_radar_para);
    ++tmp_count;
    usleep(1000*5);
  }
  return NULL;
}

void* ThreadRadarSend(void* arg, int flag)
{
  ROS_ERROR("2ThreadRadarSend");
  radar::CRadarPrep* radar_prep = (radar::CRadarPrep*)arg;
  radar_prep->can_driver[flag]->UdpSendCanData(&radar_prep->can_driver[flag]->m_radar_para);
  return NULL;
}

void* ThreadPubRadarPrep(void *arg) // ???why no prototype
{
  radar::CRadarPrep* radar_prep = (radar::CRadarPrep*)arg;
  int tmp_count = 0;
  while (ros::ok())
  {
    radar_prep->PubRadarPrep();
    ++tmp_count;
    usleep(1000*50);
  }
  return NULL;
}

void *ThreadPubDiagnose(void *arg)
{
  radar::CRadarPrep* radar_prep = (radar::CRadarPrep*)arg;   // Basic argument def.

  int tmp_count = 0;                                    // Only fot test.
  while(ros::ok())
  {
    radar_prep->PubDiagnose();
    ++tmp_count;                                                     // Only for test.
    usleep(1000);
  }
}
