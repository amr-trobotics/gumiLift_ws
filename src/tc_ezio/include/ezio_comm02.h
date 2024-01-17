#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <ifaddrs.h>
#include <unistd.h>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <vector>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <ctime>

#include "tc_msgs/msg/io_mod_msg.hpp"
#include "tc_msgs/srv/io_mod_to_man.hpp"

#include "std_msgs/msg/string.hpp"

#include "data_type/host_data.h"
#include "data_type/datatype.h"
#include "data_type/constants.h"

using namespace std::chrono_literals;
using namespace std;

#define FD_MAX_SIZE 2000 //4096000

class Ezio2 : public rclcpp::Node
{
   public:
      Ezio2();
      ~Ezio2();  

      bitInfo stbit_info_; //임시 사용. 계산을 못하겠다. ㅜㅜ

      void initialCreate(); //변수 생성 하기.
      void initialPara(); //파라미터 리셋
      bool initialComm();
      bool reconnectCallback();
      void initialOutReceive();
      void srvCommandSend(int task); //서버에 메세지 보내기
      bool srvCommandSend(int set_data_01, int set_data_02, int set_data_03, int set_data_04, int reset_01, int reset_02 ,int reset_03 ,int reset_04 );
      bool srvDataReceiv(); //서버에서 데이터 리턴 받기.
      void returnDataIoStatus(int &data_01, int &data_02, int &data_03, int &data_04); //xMessageEvent에 데이터 올려 준다.
      void bitStateToBlockState(int nModID, int nBlockID, int nbitNo, int state);
      bool outEzioSetting();
      bool return_set_out_();
   
   public:
      int socket_client_fd_; //소켓 생성자
      int send_counter_; //change couter use to Socket sending.

      int wd_data_[4]; //Block Count;

      char rev_data_[FD_MAX_SIZE];

      rclcpp::TimerBase::SharedPtr tm_srv_receive_;

      struct sockaddr_in st_srv_addr_; //Ip address and Port
};


