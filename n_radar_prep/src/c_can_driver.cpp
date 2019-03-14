/*******************************************************************/
/*                      Author: xgt                                */
/*                     Contact: xieguotao1990@126.com              */
/*               First version: 2018-05-21                         */
/*                 Last update: 2018-05-21                         */
/*******************************************************************/

/*
  Description: The cpp file for C++ class CanDriver.
  The class includes all the necessary vars and funcs for radar information receiving
  based on canet udp socket.
*/


// Include system lib.
#include<cmath>
#include <ros/ros.h>
#include <poll.h>
// Include personal lib.

#include "c_can_driver.h"


using namespace std;

CanDriver::CanDriver() // !!!fault
{
    cout << "CanDriver Class is built." << endl;

}

CanDriver::~CanDriver()
{
    close(sockfd);
    cout << "CanDriver Class is off." << endl;
}

void CanDriver::SetParameter(const char* host_ip, const char* radar_ip,const int prt_host_rev, const int prt_radar_rev, const int prt_host_send, const int prt_radar_send)
{
  //sockaddr_in hostNett = CanDriver::hostNet;
  //sockaddr_in canett = CanDriver::canet;
  memset(&hostNet, 0, sizeof(hostNet));
  hostNet.sin_family=AF_INET;
  hostNet.sin_addr.s_addr=inet_addr(host_ip); // INADDR_ANY
  hostNet.sin_port=htons(prt_host_rev); //3004

  memset(&radar_rev, 0, sizeof(radar_rev));
  radar_rev.sin_family=AF_INET;
  radar_rev.sin_addr.s_addr=inet_addr(radar_ip);   //INADDR_ANY
  radar_rev.sin_port=htons(prt_radar_rev); //7004
  printf("Parameters of CanDriver_rev were set\n");

  memset(&radar_send, 0, sizeof(radar_send));
  radar_send.sin_family=AF_INET;
  radar_send.sin_addr.s_addr=inet_addr(radar_ip);   //INADDR_ANY
  radar_send.sin_port=htons(prt_radar_send); // 7004 or 7006
  printf("Parameters of CanDrive_send were set\n");

  //raw_pub = n.advertise<sensor_msgs::PointCloud>("rawPub",10);

  //sockaddr_in my_addr;                     // my address information
  /*memset(&my_addr, 0, sizeof(my_addr));    // initialize to zeros
  my_addr.sin_family = AF_INET;            // host byte order
  my_addr.sin_port = htons(8001);          // port in network byte order
  my_addr.sin_addr.s_addr = inet_addr(host_ip);*/  //INADDR_ANY;    // automatically fill in my IP
  //return 0;
}

int CanDriver::SocketBuild()
{
  //sockaddr_in hostNett = hostNet;
  //sockfd = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);


  sockfd = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd == -1)
  {
    perror("socket");               // TODO: ROS_ERROR errno
    return 1;
  }
  int on = 1 ;
  if(setsockopt(sockfd,SOL_SOCKET,SO_REUSEADDR,(void *)&on,sizeof(int)) == -1) // !!!bind: Address already in use
  {
      perror("setsockopt");
      return 1;
  }
  if (bind(sockfd, (sockaddr *)&hostNet, sizeof(sockaddr)) == -1)
  {
    perror("bind");                 // TODO: ROS_ERROR errno
    return 1;
  }
  if (fcntl(sockfd,F_SETFL, O_NONBLOCK|FASYNC) < 0)
  {
    perror("non-block");
    return 1;
  }
  std::cout << "socket was built in SocketBuild func" << std::endl;
  /*if(bind(sockfd,(struct sockaddr *)&hostNet,sizeof(struct sockaddr_in))<0)
  {
    printf("Bind fail!\n");
    exit(1);
  }*/
  return 0;
}

int CanDriver::UdpRecCanData(const RadarParameters* radar_para) // declare every-time???
{
  static int rec_count = 0;
  //std::cout<<"radar start recevier data :"<<rec_count++<<std::endl;

  fd_set rfdset;//声明可读文件描述符集

  FD_ZERO(&rfdset);//清空文件描述集
  FD_SET(sockfd,&rfdset);//添加sockfd到rfdset

  struct timeval timeout;//等待时间
  timeout.tv_sec = 1;//秒
  timeout.tv_usec = 0;//毫秒

  int ret = 0;//返回标志
  ret = select(sockfd+1,&rfdset,NULL,NULL,&timeout);//判断sockfd是否可读
  if(ret == -1)//select出错
  {
    ROS_ERROR("radar select failure");
    return 1;
  }
  else if(ret == 0)//在等待时间内sockfd不可读
  {
    ROS_WARN("radar recv timeout");
    return 1;
  }

  if(FD_ISSET(sockfd,&rfdset))//判断sockfd在rfdset中
  {
    //printf("socckfd is rfdset\n");
  }
  //dhl
  //sockaddr_in canett = canet;
  //unsigned int addrlen;
  socklen_t addrlen= sizeof(radar_rev);
  memset(rec_buf,0,sizeof(0));
  // how to ensure rec_buf is normal, if lost too much, it is affected track at last. 1023???
  num_rec_buf =  recvfrom(sockfd, rec_buf, 1023, BLOCK, (struct sockaddr*)&radar_rev, &addrlen);  //recvfrom func receive a address and perseve origin address
  //std::cout<<"recv_buf num : "<<num_rec_buf<<std::endl;
  rec_buf[num_rec_buf] = '\0';

  if (num_rec_buf <= 0)
  {
    printf("Receive Error\n");
    return -1;
  }

  RadarDataDef(radar_para, rec_buf);

  return 0;
}

int CanDriver::UdpSendCanData(const RadarParameters* radar_para) // declare every-time???
{
   if(radar_para->radar_comp == "Delphi" && radar_para->radar_type == "rsds")
  {
    socklen_t addr_len = sizeof(struct sockaddr_in);
    char buffer[256];
    unsigned char buf20[13] = {0x08,0x00,0x00,0x00,0x20,0x00,0x08,0x00,0x00,0x9b,0x0a,0x8c,0x70};  //declared in classinit
    unsigned char buf100[65]={0x08,0x00,0x00,0x00,0x65,0x00,0x00,0x00,0x00,0x00,0x07,0x04,0xC8,
                              0x08,0x00,0x00,0x01,0x00,0x80,0x20,0x00,0xA7,0x50,0x80,0xDF,0xFE,
                              0x08,0x00,0x00,0x01,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x78,0x00,
                              0x08,0x00,0x00,0x01,0x45,0x16,0x00,0x80,0x00,0x00,0x00,0x37,0xFF,
                              0x08,0x00,0x00,0x01,0xA5,0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x00};
    unsigned char buf245[143]={0x08,0x00,0x00,0x04,0x00,0x01,0x61,0x00,0x00,0x00,0x00,0x00,0x00,
                               0x08,0x00,0x00,0x04,0x00,0x02,0x01,0x02,0x00,0x02,0x00,0x00,0x00,
                               0x08,0x00,0x00,0x04,0x00,0x30,0x00,0x00,0x00,0x00,0x00,0x01,0x00,
                               0x08,0x00,0x00,0x04,0x00,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                               0x08,0x00,0x00,0x04,0x00,0x06,0x00,0x06,0x00,0x00,0x00,0x00,0x00,
                               0x08,0x00,0x00,0x04,0x00,0x09,0x00,0x00,0x01,0x00,0x00,0x00,0x00,
                               0x08,0x00,0x00,0x04,0x05,0x01,0x00,0x00,0x01,0x00,0x00,0x09,0xC6,
                               0x08,0x00,0x00,0x04,0x05,0x02,0x00,0x40,0x00,0xFF,0x00,0x00,0x00,
                               0x08,0x00,0x00,0x04,0x05,0x11,0x44,0x45,0x46,0x47,0x48,0x49,0x50,
                               0x08,0x00,0x00,0x04,0x05,0x12,0x51,0x52,0x53,0x54,0x55,0x56,0x57};
    unsigned char buf300[13]={0x08,0x00,0x00,0x05,0x00,0x66,0x02,0x00,0x01,0x00,0x00,0x01,0x00};

    ROS_ERROR("Ready send!!!");
    int cycle = 0;
    while(ros::ok())
    {
      //msg_speed->16 bit->declare here to somewhere of buf20
      //printf("cycle : %d--\n",cycle);
      if(cycle % 4 == 0)                    // 20ms
        sendto(sockfd,buf20,sizeof(buf20),0,(struct sockaddr *)&radar_send,addr_len);
      if(cycle % 20 == 0)                   // 100ms
        sendto(sockfd,buf100,sizeof(buf100),0,(struct sockaddr *)&radar_send,addr_len);
      if(cycle % 49 == 0)                   // 245ms
        sendto(sockfd,buf245,sizeof(buf245),0,(struct sockaddr *)&radar_send,addr_len);
      if(cycle % 60 == 0)                   // 300ms
        sendto(sockfd,buf300,sizeof(buf300),0,(struct sockaddr *)&radar_send,addr_len);

      if(cycle == 300)                      // achieve 300ms another loop
      {
        cycle = 0;
      }
      cycle ++;
      usleep(5 * 1000);
    }
  }
  return 0;
}

int CanDriver::RadarDataDef(const RadarParameters* radar_para, unsigned char rec_buf[])
{
    if (radar_para->radar_comp == "Delphi" && radar_para->radar_type == "esr")
    {
        this->DelphiEsrData(radar_para, rec_buf);

    }
    else if(radar_para->radar_comp == "Delphi" && radar_para->radar_type == "rsds")
    {
        this->DelphiRsdsData(radar_para, rec_buf);
    }
    else
    {
        return  -1;
    }
    return 0;
}

int CanDriver::DelphiEsrData(const RadarParameters* radar_para, unsigned char rec_buf[])
{    
    RadarRawData temp;
    //std::cout<<"data Def"<<std::endl;
    if(num_rec_buf <= 0)
    {
        std::cout<<"recv_buf no data"<<std::endl;
        return 1;
    }
    else
    {
        //std::cout<<"have data :"<<num_rec_buf<<std::endl;
        frame_num = num_rec_buf / 13;
        std::cout<<"-------------recv_Esr_frame_num:"<<frame_num<<"-------------"<<std::endl;

        //int k = 0;

        for(int j = 0; j < frame_num; j++)   // extract each frame
        {
            if(((rec_buf[j*13+3] * 0x100) + rec_buf[j*13+4]) >= 0x0500 && ((rec_buf[j*13+3] * 0x100) + rec_buf[j*13+4]) <= 0x053F )
            {
                temp.range = (((rec_buf[j*13+7] & 0x07) * 0x100)|rec_buf[j*13+8]) * 0.1;
                //HUAI_LOG_INFO("range: %f",temp.range);
                //radar_raw_msg[k].range = (((rec_buf[j*13+7] & 0x07) * 0x100)|rec_buf[j*13+8]) * 0.1;   // range

                if((rec_buf[j*13+6] & 0x10) >> 4 == 0)   // angle/+-
                {
                  temp.angle = ((((rec_buf[j*13+6] & 0x1F) * 0x100) | (rec_buf[j*13+7] & 0xF8)) >> 3) * 0.1;
                  //radar_raw_msg[k].angle = ((((rec_buf[j*13+6] & 0x1F) * 0x100) | (rec_buf[j*13+7] & 0xF8)) >> 3) * 0.1;// test、if+
                }
                else
                {
                  temp.angle = -(1024 - ((((rec_buf[j*13+6] & 0x1F) * 0x100) | (rec_buf[j*13+7] & 0xF8)) >> 3)) * 0.1; //if -
                  //radar_raw_msg[k].angle = -(1024 - ((((rec_buf[j*13+6] & 0x1F) * 0x100) | (rec_buf[j*13+7] & 0xF8)) >> 3)) * 0.1; //if -
                }
                if((rec_buf[j*13+11] & 0x20) >> 5 == 0)   // range_rate /+-
                {
                  temp.range_rate = (((rec_buf[j*13+11] & 0x3F) * 0x100) | (rec_buf[j*13+12])) * 0.01; //test、if+
                  //radar_raw_msg[k].range_rate = (((rec_buf[j*13+11] & 0x3F) * 0x100) | (rec_buf[j*13+12])) * 0.01; //test、if+
                }
                else
                {
                  temp.range_rate = -(16384 - (((rec_buf[j*13+11] & 0x3F) * 0x100) | (rec_buf[j*13+12]))) * 0.01; //if-
                  //radar_raw_msg[k].range_rate = -(16384 - (((rec_buf[j*13+11] & 0x3F) * 0x100) | (rec_buf[j*13+12]))) * 0.01; //if-
                }

                radar_raw_msg.push_back(temp);

                //++k;
            }
            else
            {
                ;
            }

        }
        this->RadarObjNoiseFilter(radar_para);
        this->RadarObjCoordTrans(radar_para);

        boost::mutex mutex_rec;
        boost::unique_lock<boost::mutex> lock(mutex_rec); // mutex the shared_neicun, insure after the for_loop below, then next step

        for(int i = 0; i < radar_obj_msg_vehicle.size(); i++)
        {

            point3.x = radar_obj_msg_vehicle[i].x;
            point3.y = radar_obj_msg_vehicle[i].y;
            msg_radar_point3_cloud.radar_prep.push_back(point3); // give to msg_radar_points_cloud to c_radar_prep for whole pub to other-node
            points.x = point3.x; //1024 * rand () / (RAND_MAX + 1.0f); //
            points.y = point3.y; //1024 * rand () / (RAND_MAX + 1.0f); //
            points.z = point3.z; //1024 * rand () / (RAND_MAX + 1.0f); //
            cloud_output.points.push_back(points); // for single pub in c_radar_prep
            ++i;
        }

        radar_obj_msg_vehicle.clear(); // clear in time if not core dumped
    }
    return 0;
}

int CanDriver::DelphiRsdsData(const RadarParameters* radar_para, unsigned char rec_buf[])
{
    if(num_rec_buf <= 0)
    {
        std::cout<<"recv_buf no data"<<std::endl;
        return 1;
    }
    else
    {
        frame_num = num_rec_buf / 13;
        std::cout<<"-------------recv_Rsds_frame_num:"<<frame_num<<"-------------"<<std::endl;
        RadarRawData temp;
        for(int j = 0; j < frame_num; j++)   // extract each frame
        {
//            if(((rec_buf[j*13+3] * 0x100) + rec_buf[j*13+4]) >= 1280 && ((rec_buf[j*13+3] * 0x100) + rec_buf[j*13+4]) <= 1343 ) //500-53F in driver side
//            {
//                temp.range = (((rec_buf[j*13+9]) * 0x100)+rec_buf[j*13+10]) * 0.0078125;   // range
//                temp.install_side = "left";
//                //std::cout<<"-------id: "<< j << "-" <<temp.install_side << "------Rsds_raw_vehicle:"<<  temp.range  <<"-------------"<<std::endl;
//                if((rec_buf[j*13+7] & 0x80) >> 7 == 0)   // angle/+-
//                {
//                    temp.angle = (((rec_buf[j*13+7]) * 0x100) + rec_buf[j*13+8]) * 0.0078125;// test、if+
//                    HUAI_LOG_INFO("range: %f",temp.range);
//                    float temp_x = temp.range * cos(temp.angle * M_PI / 180.0);
//                    HUAI_LOG_INFO("x_position: %f",temp_x);
//                    //std::cout << "angle : " <<angle[n] << std::endl;
//                }
//                else
//                {
//                    temp.angle = -(65536 - (((rec_buf[j*13+7]) * 0x100) + rec_buf[j*13+8])) * 0.0078125; //if -
//                    HUAI_LOG_INFO("range: %f",temp.range);
//                    float temp_x = temp.range * cos(temp.angle * M_PI / 180.0);
//                    HUAI_LOG_INFO("x_position: %f",temp_x);
//                    //                    std::cout << "angle : " <<angle[n] << std::endl;
//                }
//                //std::cout <<"highest : "<< (rec_buf[j*13+11] & 0x80) >> 7 << std::endl;
//                if((rec_buf[j*13+11] & 0x80) >> 7 == 0)   // range_rate /+-
//                {
//                    temp.range_rate = (((rec_buf[j*13+11]) * 0x100) + (rec_buf[j*13+12])) * 0.0078125; //test、if+
//                }
//                else
//                {
//                    temp.range_rate = -(65536 - (((rec_buf[j*13+11]) * 0x100) + (rec_buf[j*13+12]))) * 0.0078125; //if-
//                }
//                radar_raw_msg.push_back(temp);

//            }
            if(((rec_buf[j*13+3] * 0x100) + rec_buf[j*13+4]) >= 1536 && ((rec_buf[j*13+3] * 0x100) + rec_buf[j*13+4]) <= 1599 ) //600-63F passenger side
            {
                temp.range = (((rec_buf[j*13+9]) * 0x100)+rec_buf[j*13+10]) * 0.0078125;   // range
                temp.install_side = "right";
                //std::cout<<"-------id: "<< j << "-" <<temp.install_side << "------Rsds_raw_vehicle:"<<  temp.range  <<"-------------"<<std::endl;
                //ROS_WARN("right______range : %f", temp.range );

                if((rec_buf[j*13+7] & 0x80) >> 7 == 0)   // angle/+-
                {
                    temp.angle = (((rec_buf[j*13+7]) * 0x100) + rec_buf[j*13+8]) * 0.0078125;// test、if+
                    //                    std::cout << "angle : " <<angle[n] << std::endl;
                }
                else
                {
                    temp.angle = -(65536 - (((rec_buf[j*13+7]) * 0x100) + rec_buf[j*13+8])) * 0.0078125; //if -
                    //                    std::cout << "angle : " <<angle[n] << std::endl;
                }
                if((rec_buf[j*13+11] & 0x80) >> 7 == 0)   // range_rate /+-
                {
                    temp.range_rate = (((rec_buf[j*13+11]) * 0x100) + (rec_buf[j*13+12])) * 0.0078125; //test、if+
                }
                else
                {
                    temp.range_rate = -(65536 - (((rec_buf[j*13+11]) * 0x100) + (rec_buf[j*13+12]))) * 0.0078125; //if-
                }
                radar_raw_msg.push_back(temp);
            }
            else
            {
              ;
            }

        }

        this->RadarObjNoiseFilter(radar_para);
        //this->RadarObjCoordTrans(radar_para);

        boost::mutex mutex_rec;
        boost::unique_lock<boost::mutex> lock(mutex_rec); // mutex the shared_neicun, insure after the for_loop below, then next step
        for(int i = 0; i < radar_obj_msg.size(); i++)
        {
          //point3.x = radar_obj_msg[i].x;
          //point3.y = radar_obj_msg[i].x;  // for left_calibration
          point3.y = radar_obj_msg[i].x;  // for right_calibration

          //point3.y = radar_obj_msg[i].y;
          //point3.x = -radar_obj_msg[i].y;  // for left_calibration
          point3.x = radar_obj_msg[i].y;  // for right_calibration

          //point3.x = radar_obj_msg_vehicle[j].x;
          //point3.y = radar_obj_msg_vehicle[j].y;
          msg_radar_point3_cloud.radar_prep.push_back(point3); // give to msg_radar_points_cloud to c_radar_prep for whole pub to other-node
          points.x = point3.x;
          points.y = point3.y;
          points.z = point3.z;
          cloud_output.points.push_back(points); // for c_radar_prep pub independently
        }
        radar_obj_msg.clear();
        //radar_obj_msg_vehicle.clear(); // clear in time if not core dumped
    }
    return 0;
}

int CanDriver::RadarObjNoiseFilter(const RadarParameters* radar_para)
{
    if(radar_para->radar_comp == "Delphi" && radar_para->radar_type == "esr")
    {
      RadarObjData temp;
      for(int i = 0; i < radar_raw_msg.size(); i++)
      {
        if((radar_raw_msg[i].range != 0) && (radar_raw_msg[i].angle != 0)) // filter zero-targe
        {

          temp.x = radar_raw_msg[i].range * cos(radar_raw_msg[i].angle * M_PI / 180.0);
          std::cout<<"id: "<< i << "------ESR_obj_X "<<  temp.x  <<"-------------"<<std::endl;
          temp.y = radar_raw_msg[i].range * sin(radar_raw_msg[i].angle * M_PI / 180.0);
          temp.vx = radar_raw_msg[i].range_rate * cos(radar_raw_msg[i].angle * M_PI / 180.0);
          temp.vy = radar_raw_msg[i].range_rate * sin(radar_raw_msg[i].angle * M_PI / 180.0);
          radar_obj_msg.push_back(temp);
        }
        else
        {
          ;
        }
      }
      radar_raw_msg.clear(); // clear in time if not core dumped
    }
    if(radar_para->radar_comp == "Delphi" && radar_para->radar_type == "rsds")
    {
      RadarObjData temp;
      for(int i = 0; i < radar_raw_msg.size(); i++)
      {
        if((radar_raw_msg[i].range != 0) && (radar_raw_msg[i].angle != 0)) // filter zero-targe
        {

          temp.x = radar_raw_msg[i].range * cos(radar_raw_msg[i].angle * M_PI / 180.0);
          temp.y = radar_raw_msg[i].range * sin(radar_raw_msg[i].angle * M_PI / 180.0);
          temp.vx = radar_raw_msg[i].range_rate * cos(radar_raw_msg[i].angle * M_PI / 180.0);
          temp.vy = radar_raw_msg[i].range_rate * sin(radar_raw_msg[i].angle * M_PI / 180.0);
          temp.install_side = radar_raw_msg[i].install_side;
          radar_obj_msg.push_back(temp);
        }
        else
        {
          ;
        }
      }
      radar_raw_msg.clear(); // clear in time if not core dumped, out of for-loop
    }
    else
    {
      ;
    }
    return 0;
}

int CanDriver::RadarObjCoordTrans(const RadarParameters* radar_para)
{
  if(radar_para->radar_comp == "Delphi" && radar_para->radar_type == "esr")
  {
    RadarObjData temp;
    for(int i = 0; i < radar_obj_msg.size(); i++)
    {
      temp.x=radar_para->install_x + radar_obj_msg[i].x; // distance_x need read num from ini
      temp.y=-radar_obj_msg[i].y;
      radar_obj_msg_vehicle.push_back(temp);
    }
    radar_obj_msg.clear(); // clear in time if not core dumped
  }

  if(radar_para->radar_comp == "Delphi" && radar_para->radar_type == "rsds")
  {
    RadarObjData temp;
    for(int i = 0; i < radar_obj_msg.size(); i++)
    {
      if(radar_obj_msg[i].install_side == "right")
      {
        temp.x= -((radar_para->install_b) + (radar_para->install_sin_alfa) * radar_obj_msg[i].x + (radar_para->install_cos_alfa) * radar_obj_msg[i].y); // sin_alfa read from ini
        temp.y= -((radar_para->install_a) + (radar_para->install_cos_alfa) * radar_obj_msg[i].x - (radar_para->install_sin_alfa) * radar_obj_msg[i].y); // cos_alfa read from ini
        radar_obj_msg_vehicle.push_back(temp);
      }
      if(radar_obj_msg[i].install_side == "left")
      {
        temp.x= -((radar_para->install_b) + (radar_para->install_sin_alfa) * radar_obj_msg[i].x - (radar_para->install_cos_alfa) * radar_obj_msg[i].y); // sin_alfa read from ini
        temp.y= (radar_para->install_a) + (radar_para->install_cos_alfa) * radar_obj_msg[i].x + (radar_para->install_sin_alfa) * radar_obj_msg[i].y; // cos_alfa read from ini, no negtive
        radar_obj_msg_vehicle.push_back(temp);
      }
      else
      {
        ;
      }

    }
    radar_obj_msg.clear(); // clear in time if not core dumped
  }
  else
  {
    ;
  }
  return 0;
}

void CanDriver::CloseUdpCanet(const RadarParameters* radar_para)
{
    close(sockfd);
}




