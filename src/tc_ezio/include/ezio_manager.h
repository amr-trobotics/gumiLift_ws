#include <chrono>
#include <memory>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <ifaddrs.h>
#include <unistd.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>
#include <ctime>
#include <mutex>
#include <cmath>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <sys/time.h>
#include <functional>
#include <thread>
#include <sys/utsname.h>
#include <cstring>

#include "tc_msgs/msg/io_mod_msg.hpp"
#include "tc_msgs/srv/io_header.hpp"
#include "tc_msgs/srv/io.hpp"
#include "std_msgs/msg/int32.hpp"

#include "data_type/host_data.h"
#include "data_type/datatype.h"
#include "data_type/constants.h"
#include "data_type/tr_share_enum.hpp"

#include "ezio_comm01.h"
#include "ezio_comm02.h"
#include "ezio_comm03.h"
#include "ezio_comm04.h"
#include "ezio_comm05.h"


using namespace std::chrono_literals;
using std::placeholders::_1;

#define FD_MAX_SIZE 2000 //4096000

#define EZIO_01_UNCONNECTED 3109
#define EZIO_02_UNCONNECTED 3110
#define EZIO_03_UNCONNECTED 3111
#define EZIO_04_UNCONNECTED 3112
#define EZIO_05_UNCONNECTED 3113
#define EZIO_06_UNCONNECTED 3114

#define EZIO_01_WRITING_FAIL 3115
#define EZIO_02_WRITING_FAIL 3116
#define EZIO_03_WRITING_FAIL 3117
#define EZIO_04_WRITING_FAIL 3118
#define EZIO_05_WRITING_FAIL 3119
#define EZIO_06_WRITING_FAIL 3120

#define EZIO_01_READING_FAIL 3121
#define EZIO_02_READING_FAIL 3122
#define EZIO_03_READING_FAIL 3123
#define EZIO_04_READING_FAIL 3124
#define EZIO_05_READING_FAIL 3125
#define EZIO_06_READING_FAIL 3126


class EzioManager : public rclcpp::Node
{
   public:
      EzioManager();
      ~EzioManager();  

      char m_AbsPath[255];
      bool return_set_out_;
      bool return_set_out_id1_;
      bool return_set_out_id2_;
      bool return_set_out_id3_;
      bool return_set_out_id4_;
      bool return_set_out_id5_;

      bool srv_event_;
      bool ezio01_received_flag_;
      bool ezio02_received_flag_;
      bool ezio03_received_flag_;
      bool ezio04_received_flag_;
      bool ezio05_received_flag_;

      bool ezio01_receivedOUT_flag_;
      bool ezio02_receivedOUT_flag_;
      bool ezio03_receivedOUT_flag_;
      bool ezio04_receivedOUT_flag_;
      bool ezio05_receivedOUT_flag_;


      bool ezio01_comm_flag_;
      bool ezio02_comm_flag_;
      bool ezio03_comm_flag_;
      bool ezio04_comm_flag_;
      bool ezio05_comm_flag_;


      int error01_counter_;
      int error02_counter_;
      int error03_counter_;
      int error04_counter_;
      int error05_counter_;


      ErrorInfo st_error_;                      // Log error

      Ezio1 ezio_id01_;
      Ezio2 ezio_id02_;
      Ezio3 ezio_id03_;
      Ezio4 ezio_id04_;
      Ezio5 ezio_id05_;

      thread ezio_thread1_;
      thread ezio_thread2_;
      thread ezio_thread3_;
      thread ezio_thread4_;
      thread ezio_thread5_;

      std::vector<iomod_Data> io_out_setting_; //연속으로 IO Module을 받을 경우 문제가 있어 배열 처리 하여 하나씩 처리한다.
      std::vector<iomod_Data> io_out_array_setting_; //연속으로 IO Module을 받을 경우 문제가 있어 배열 처리 하여 하나씩 처리한다.
      std::vector<int> io_out_array_size_;
      //In_updated.
      typedef struct _IoModuleData{
         int nBlock01;
         int nBlock02;
         int nBlock03;
         int nBlock04;
         int nResBlock01;
         int nResBlock02;
         int nResBlock03;
         int nResBlock04;
      }IoModuleData;

      typedef struct _IoModuleInfo{
         IoModuleData ID[6];
      }IoModuleInfo;

      IoModuleInfo in_data_;
      IoModuleInfo out_data_;
      
      

   public:
      void initialEzioComm1();
      void initialEzioComm2();
      void initialEzioComm3();
      void initialEzioComm4();
      void initialEzioComm5();
      void scanCommCallback();

      void scanEzioCallback(); //IO Module spin();

      void inReceivEzioID01();
      void inReceivEzioID03();
      void inReceivEzioID04();
      void inReceivEzioID05();

      void outReceivEzioID02();
      void outReceivEzioID03();
      void outReceivEzioID04();
      void outReceivEzioID05();

      void DataEzioToPub();
      void outWriteToEzio();
      void arrayOutWriteToEzio();
      void selectEzio(int id, int state);
      void arraySelectEzio(int arry_id[], int array_state[], int size);
      void ioArrayServer();
      void ioServer();

          // Log Function
      void loggingBallBack();
      void coordLogging(char* c_strName);
      void tacktimeLogging(char* c_strName, char* c_strData);
      void log_err(int nErr);
      void taskLogging(Tacklist& _Tacklist);
      void logSys(int _Type, const char *frm,...);
      std::string logCreateDirectory(string foldername);
      int rmdirs(const char *path, int force);
      void logRemoveFolder(const char *path);
      // Time Function
      unsigned int getTickCount();
      unsigned long elaspseTime(unsigned int dwStartTickCount);
      bool getTimeOut(unsigned int  dwStartTickCount, int uMilliSecound);
   
   public:
      //File processing read about Index of Io Moudle.
      void initialFileRead();
      void fileReadSystemEzio();
      std::ifstream file_io_module_read_; 
      int input_io_module_counter_; //io bit count;

      iomod_info st_io_mod_info_;
      IoModuleInfo st_mod_event_data_; //xNetComm에서 올라오는 데이터 수집을 한다.

   public:

      rclcpp::TimerBase::SharedPtr tm_io_module_; 
      rclcpp::TimerBase::SharedPtr tm_check_; 
      
      rclcpp::Publisher<tc_msgs::msg::IoModMsg>::SharedPtr pub_ezio_; //IO 상태를 보고.
      rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_ezio_alarm_;

      rclcpp::Service<tc_msgs::srv::IoHeader>::SharedPtr io_service_; //change to Io status
      rclcpp::Service<tc_msgs::srv::Io>::SharedPtr io_array_service_; //change to Io status 
      
   public:

};

