/*******************************************************************/
/*                      Author: xgt                                */
/*                     Contact: xieguotao1990@126.com              */
/*               First version: 2018-05-10                         */
/*                 Last update: 2018-05-22                         */
/*******************************************************************/

/*
  Description: The cpp file for C++ class CRadarPrep.
  The class includes all the necessary vars and funcs for radar based
  estimating.
*/


// Include system lib.
#include <Eigen/Dense>
// Include personal lib.
#include "c_radar_prep.h"
#include <boost/bind.hpp>
//#include <Eigen/Dense>

#define THRESHOLD_VALUE 0.50

#define TRACK_TRAJECTORY_NUM 5

namespace radar
{
using namespace std;
// Func: Class init.
int CRadarPrep::radar_nums = 0;
void CRadarPrep::ClassInit(string cfg_file_path)
{
  //???how much need init
  pub_radar_prep = n.advertise<msg_radar_prep::msg_radar_prep>("msg_radar_prep", 500); // Pub init, msg_radar_prep named for pub
  pub_diagnose_prep = n.advertise<msg_radar_prep::msg_diagn_radar_prep>("msg_radar_diagnose",500); //publish diagnose
  sub_msg_v_state = n.subscribe<msg_v_state::msg_v_state>("/msg_v_state_pred", 20, boost::bind(& CRadarPrep::AcqVehicleData, this, _1));       // ROS subscriber init.

  pub_test_point = n_test.advertise<sensor_msgs::PointCloud>("test_points", 500);       // Pub init.
  pub_radar_esr = n_test.advertise<sensor_msgs::PointCloud>("radar_esr",500);
  pub_radar_rsds = n_test.advertise<sensor_msgs::PointCloud>("radar_rsds",500);

  OperatorIni* p_Radar_param = new OperatorIni(); // get param from config file
  p_Radar_param->ReadINI(cfg_file_path); // open file
  int num_radar = std::atoi(p_Radar_param->GetValue("NUM_RADAR","num_radar").c_str());
  radar::CRadarPrep::radar_nums = num_radar;
  std::string host_ip = p_Radar_param->GetValue("host_ip", "hostIp");

  string prefix_radar = "RADAR";
  string temp_node;
  string temp_ip;
  // tempt
  //int port_host_send, port_radar_send; // same as cfg.ini, do not need in RadarParameters
  //int port_host_rev, port_radar_rev;
  int num = 0;
  cout << " num_radar ***: "  << num_radar << endl;
  for(int i = 0 ; i < num_radar ; ++i,num++)
  {
    sensor_msgs::PointCloud temp;
    each_radar_cloud.emplace_back(temp); //???what is the effect,init??
    temp_node = prefix_radar + to_string(num);
    can_driver.emplace_back(new CanDriver());
    temp_ip = p_Radar_param->GetValue(temp_node,"radar_ip");
    //can_driver[i]->n = this->n;

    can_driver[i]->m_radar_para.port_host_send = std::atoi(p_Radar_param->GetValue(temp_node,"port_host_send").c_str());
    can_driver[i]->m_radar_para.port_radar_send = std::atoi(p_Radar_param->GetValue(temp_node,"port_radar_send").c_str());
    can_driver[i]->m_radar_para.port_host_rev = std::atoi(p_Radar_param->GetValue(temp_node,"port_host_rev").c_str());
    can_driver[i]->m_radar_para.port_radar_rev = std::atoi(p_Radar_param->GetValue(temp_node,"port_radar_rev").c_str());
    can_driver[i]->SetParameter(host_ip.c_str(),temp_ip.c_str(),can_driver[i]->m_radar_para.port_host_rev, can_driver[i]->m_radar_para.port_radar_rev,
                                can_driver[i]->m_radar_para.port_host_send, can_driver[i]->m_radar_para.port_radar_send); //rev:c_can_driver.setparameter

    //can_driver[i]->SetParameter(host_ip.c_str(),temp_ip.c_str(),can_driver[i]->radar_para.port_host_send, can_driver[i]->radar_para.port_radar_send); //send:c_can_driver.setparameter

    can_driver[i]->SocketBuild();
    // to init radar equipment diagnose in UdpRecCandata
    //can_driver[i]->radar_para.radar_ip = temp_ip;
    //can_driver[i].radar_para.port_radar_rev = port_radar_rev;
    //can_driver[i]->hostip = host_ip;
    //can_driver[i]->hostport = port_host_rev;
    //can_driver[i]->radar_diagnose.equip_num = i+1; // to init lidar equiment  diagnose

    //??? lidar_udp[i]->rate_cycle_index = std::atoi(p_Lidar_param->GetValue(temp_node,"rate_cycle_index").c_str());
    //    GetFactor(p_Lidar_param->GetValue(temp_node,"factor"),lidar_udp[i]->my_lidar_para.factor);
    can_driver[i]->m_radar_para.radar_comp = p_Radar_param->GetValue(temp_node,"radar_comp");
//    std::cout << "m_radar_para.radar_comp:" << can_driver[i]->m_radar_para.radar_comp << std::endl;
    can_driver[i]->m_radar_para.radar_type = p_Radar_param->GetValue(temp_node,"radar_type");
    if(can_driver[i]->m_radar_para.radar_type == "esr")
    {
//      can_driver[i]->m_radar_para.install_x = std::atoi(p_Radar_param->GetValue(temp_node,"install_x").c_str());
        can_driver[i]->m_radar_para.install_x = float(std::atof(p_Radar_param->GetValue(temp_node,"install_x").c_str()));
    }
    if(can_driver[i]->m_radar_para.radar_type == "rsds")
    {
        can_driver[i]->m_radar_para.install_a = float(std::atof(p_Radar_param->GetValue(temp_node,"install_a").c_str())); //fault
        can_driver[i]->m_radar_para.install_b = float(std::atof(p_Radar_param->GetValue(temp_node,"install_b").c_str()));
        can_driver[i]->m_radar_para.install_sin_alfa = float(std::atof(p_Radar_param->GetValue(temp_node,"install_sin_alfa").c_str()));
        can_driver[i]->m_radar_para.install_cos_alfa = float(std::atof(p_Radar_param->GetValue(temp_node,"install_cos_alfa").c_str()));
    }

    //    lidar_udp[i]->my_lidar_para.lidar_mode = p_Lidar_param->GetValue(temp_node,"lidar_mode");
    //    lidar_udp[i]->my_lidar_para.fixPos = (LIDAR_INDEX)std::atoi(p_Lidar_param->GetValue(temp_node,"fixPos").c_str());

  }
  std::cout << "Socket build function was finished in classnit func!!" << std::endl;

  delete p_Radar_param;
  ROS_ERROR("Initionalization was finished!!" );

  /*Initialize the canetUdp receiving and sending
  char host_ip[] = "127.0.0.1";                                      //the ip address of host    192.168.1.1
  char canet_ip1[] = "127.0.0.1";                                     //the ip address of canet1
  char canet_ip2[] = "192.168.1.3";                                     //the ip address of canet2
  int prt1[3] = {4001, 4002, 4003};                                     //the port of canet

  //This is for the module based style
  for(int i = 0; i < radar_nums; i++)
  { init can_driver objects
    can_driver[i].SetParameter(host_ip, canet_ip1, prt1[i]);
    can_driver[i].SocketBuild();
    can_driver[i].radar_para.num = i;
  }
    //init the radar parameters in the system
    can_driver[0].radar_para.radar_comp = "Delphi";
    can_driver[0].radar_para.radar_type = "Esr";
    can_driver[0].radar_para.num = 0;

  can_driver[1].radar_para.radar_comp = "Delphi";
  can_driver[1].radar_para.radar_type = "Rsds";
  can_driver[1].radar_para.num = 1;

  can_driver[2].radar_para.radar_comp = "Delphi";
  can_driver[2].radar_para.radar_type = "Rsds";
  can_driver[2].radar_para.num = 2;

  can_driver[0].SetParameter(host_ip, canet_ip1, prt1[0]);         // for the radar in the front
  can_driver[1].SetParameter(host_ip, canet_ip1, prt1[0]);         // for the radar in the front
  can_driver[2].SetParameter(host_ip, canet_ip1, prt1[0]);         // for the radar in the front

  my_canet_udp_front.SetParameter(host_ip, canet_ip1, prt1[0]);         // for the radar in the front
  my_canet_udp_rear_left.SetParameter(host_ip, canet_ip1, prt1[1]);	    // for the radar in the rear left
  my_canet_udp_rear_right.SetParameter(host_ip, canet_ip1, prt1[2]);	// for the radar in the rear right

  my_canet_udp_front.SocketBuild();
  my_canet_udp_rear_left.SocketBuild();
  my_canet_udp_rear_right.SocketBuild();

  //Discrete LTI projectile motion, measuring position only
  delete opt_cfg_file;*/

}

//data prep
void CRadarPrep::RadarDataPrep()
{

}

// Func: Send vehicle data to the radar.
//void CRadarPrep::SendVehicleData()
//{

//	// Send vehicle information to radars.
//	for (int i = 0; i < NUM_RADAR; i = i + 1)
//	{
//    can_driver[i].send_buf[0] = 0;
//    can_driver[i].send_buf[1] = 0x01;
//    can_driver[i].send_buf[2] = 0x02;
//    can_driver[i].send_buf[3] = 0x03;
//    can_driver[i].send_buf[4] = 0x04;
//    can_driver[i].UdpSendCanData();
//	}

//}

// Func: Get radar raw data from the ethernet.

//pub radar data prep;
void CRadarPrep::PubRadarPrep()
{
  /*to do*/
  //  pub_prep.publish(msg_radar_prep);
  if(!msg_radar_prep_data.radar_prep.empty())  // ???how radar_prep,empty() came out| whether empty, if not do clear()
  {
    msg_radar_prep_data.radar_prep.clear();
  }
  for (int i = 0; i < radar_nums; i = i + 1)
  {
    boost::mutex mergeData_mutex;   //the mutex is used to merge all data of radar | another radar need to wait after one-radar pubthread end
    boost::unique_lock<boost::mutex> lock(mergeData_mutex); // protect the shared neicun

    if(can_driver[i]->msg_radar_point3_cloud.radar_prep.size() != 0) // pub independently
    {
      msg_radar_prep_data.radar_prep.insert(msg_radar_prep_data.radar_prep.end(),
                                            can_driver[i]->msg_radar_point3_cloud.radar_prep.begin(),
                                            can_driver[i]->msg_radar_point3_cloud.radar_prep.end()
                                            ); // put msg to other node
      cloud_output.points.insert(cloud_output.points.end(),
                                 can_driver[i]->cloud_output.points.begin(),
                                 can_driver[i]->cloud_output.points.end()
                                 ); // for whole viz

      each_radar_cloud[i] = can_driver[i]->cloud_output;
      static uint32 id = 1;
      each_radar_cloud[i].header.seq = id;
      each_radar_cloud[i].header.stamp = ros::Time::now();
      each_radar_cloud[i].header.frame_id = "base_link";

      switch (i) { // pub independently
      case 0:
        pub_radar_esr.publish(each_radar_cloud[i]);
        break;
      case 1:
        pub_radar_rsds.publish(each_radar_cloud[i]);
        break;
      }
      can_driver[i]->cloud_output.points.clear();
      can_driver[i]->msg_radar_point3_cloud.radar_prep.clear();
      each_radar_cloud[i].points.clear();
    }
    else
    {
      ROS_WARN_STREAM("now time the frame " << i  << " haven't data");
      // HUAI_LOG_DEBUG("now time the frame %d haven't data",i);
    }
    if(msg_radar_prep_data.radar_prep.size() == 0) return;  //frist must to judge
    static uint32 id = 1;
    cloud_output.header.seq = id;
    cloud_output.header.stamp = ros::Time::now();
    cloud_output.header.frame_id = "base_link";
    pub_test_point.publish(cloud_output); // pub rviz together
    cloud_output.points.clear(); // clear whole pub point each time

    cout << "size: " << msg_radar_prep_data.radar_prep.size() << endl;
    pub_radar_prep.publish(msg_radar_prep_data); // pub msg to other node no rviz

    id++;
    msg_radar_prep_data.radar_prep.clear();
  }
}

void CRadarPrep::PubDiagnose()
{
  //pub_diagnose_prep.publish(radar_prep_diagnose);
}

// Func: Acquire vehicle state from ethernet.
void CRadarPrep::AcqVehicleData(const msg_v_state::msg_v_stateConstPtr & msg)
{
  /*to do: */
  static int tmp_count = 0;
  for(int i = 0 ; i < radar_nums; ++i)
  {
    //can_driver[i].speed = msg->speed; // recieve speed from msg_v_state.msg->speed, and speed declared in candriver
  }
  ROS_INFO("Vehicle data received: %d", tmp_count++);

}

CRadarPrep::~CRadarPrep()
{
  if(!can_driver.empty())
  {
    for(int i = 0; i <  static_cast<int>(can_driver.size()); i++)
    {
      CanDriver *p = can_driver[i];   // class CanDriver layer also deleted
      delete p;
    }
  }

}

}  // namespace radar


