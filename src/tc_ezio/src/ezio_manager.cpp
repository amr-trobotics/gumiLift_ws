#include "ezio_manager.h"

using namespace std;
//생성자
EzioManager::EzioManager()
        : Node("EzioManager")
{   
    return_set_out_ = true;
    srv_event_ = false;

    error01_counter_ = 0;
    error02_counter_ = 0;
    error03_counter_ = 0;
    error04_counter_ = 0;
    error05_counter_ = 0;

    ezio01_comm_flag_ = false;
    ezio02_comm_flag_ = false;
    ezio03_comm_flag_ = false;
    ezio04_comm_flag_ = false;
    ezio05_comm_flag_ = false;

    ezio01_received_flag_ = false;
    ezio02_received_flag_ = false;
    ezio03_received_flag_ = false;
    ezio04_received_flag_ = false;
    ezio05_received_flag_ = false;

    ezio01_receivedOUT_flag_ = false;
    ezio02_receivedOUT_flag_ = false;
    ezio03_receivedOUT_flag_ = false;
    ezio04_receivedOUT_flag_ = false;
    ezio05_receivedOUT_flag_ = false;

    char* res = realpath(".", m_AbsPath);
    RCLCPP_INFO(get_logger(), "EzioManager Start path = %s",m_AbsPath );

    initialFileRead();
    ioArrayServer();
    ioServer();

    pub_ezio_ = this->create_publisher<tc_msgs::msg::IoModMsg>("InIOStates",rclcpp::QoS(rclcpp::KeepLast(10))); 
    pub_ezio_alarm_ = this->create_publisher<std_msgs::msg::Int32>("amr_alarms",rclcpp::QoS(rclcpp::KeepLast(10))); 

    tm_io_module_ = this->create_wall_timer(
                        100ms, std::bind(&EzioManager::scanEzioCallback, this));
    tm_check_ = this->create_wall_timer(
                        2000ms, std::bind(&EzioManager::scanCommCallback, this));

    ezio_thread1_ = thread(&EzioManager::initialEzioComm1, this);
    ezio_thread2_ = thread(&EzioManager::initialEzioComm2, this);
    ezio_thread3_ = thread(&EzioManager::initialEzioComm3, this);
    ezio_thread4_ = thread(&EzioManager::initialEzioComm4, this);
    ezio_thread5_ = thread(&EzioManager::initialEzioComm5, this);

    RCLCPP_INFO(get_logger(),"EzioManager End");
    

}
//소멸자
EzioManager::~EzioManager()
{
    io_out_setting_.clear(); //삭제를 한다.
    io_out_array_setting_.clear();
    io_out_array_size_.clear();
}
void EzioManager::initialEzioComm1()
{
    //Ezio01
    ezio01_comm_flag_ = ezio_id01_.initialComm();
    if(ezio01_comm_flag_)
    {
        logSys(TrShareEnum::log_seq, "[Network] ezioID01 Connected");
    }
        
}
void EzioManager::initialEzioComm2()
{
    //Ezio02
    ezio02_comm_flag_ = ezio_id02_.initialComm();
    if(ezio02_comm_flag_)
    {
        logSys(TrShareEnum::log_seq, "[Network] ezioID02 Connected");
        
        ezio_id02_.initialOutReceive();
        RCLCPP_INFO(get_logger(), "ezioID02 initialOutReceive");
    }
    
}
void EzioManager::initialEzioComm3()
{
    //Ezio03
    ezio03_comm_flag_ = ezio_id03_.initialComm();
    if(ezio03_comm_flag_)
    {
        logSys(TrShareEnum::log_seq, "[Network] ezioID03 Connected");

        ezio_id03_.initialOutReceive();
        RCLCPP_INFO(get_logger(), "ezioID03 initialOutReceive");
    }
}
void EzioManager::initialEzioComm4()
{
    //Ezio04
    ezio04_comm_flag_ = ezio_id04_.initialComm();
    if(ezio04_comm_flag_)
    {
        logSys(TrShareEnum::log_seq, "[Network] ezioID04 Connected");

        ezio_id04_.initialOutReceive();
        RCLCPP_INFO(get_logger(), "ezioID04 initialOutReceive");
    }
    
}
void EzioManager::initialEzioComm5()
{
    //Ezio05
    ezio05_comm_flag_ = ezio_id05_.initialComm();
    if(ezio05_comm_flag_)
    {
        logSys(TrShareEnum::log_seq, "[Network] ezioID05 Connected");

        ezio_id05_.initialOutReceive();
        RCLCPP_INFO(get_logger(), "ezioID05 initialOutReceive");
    }
}

void EzioManager::scanCommCallback()
{
    //RCLCPP_INFO(get_logger(), "scanCommCallback");
    if (error01_counter_<3)
    {
        if (!ezio_id01_.reconnectCallback())
        {
            this_thread::sleep_for(chrono::milliseconds(5));
            ezio_thread1_.join();
            ezio_thread1_ = thread(&EzioManager::initialEzioComm1, this);

            if (error01_counter_ == 2 && !ezio01_comm_flag_)
            {
                logSys(TrShareEnum::log_err, "[Network] ezioID01 UnConnected");

                auto outMessage = std::make_unique<std_msgs::msg::Int32>(); 
                    outMessage->data = EZIO_01_UNCONNECTED;
                    //Topic write.
                    RCLCPP_INFO(get_logger(), "Topic amr_alarm write: %d", outMessage->data);
                    pub_ezio_alarm_->publish(std::move(outMessage));    
            }

            error01_counter_++;
        }
        else
        {
            error01_counter_ = 0;
        }
    }
    else if (!ezio01_comm_flag_)
    {
        logSys(TrShareEnum::log_err, "[Network] ezioID01 UnConnected");
    }

    if (error02_counter_<3)
    {
        if (!ezio_id02_.reconnectCallback())
        {
            this_thread::sleep_for(chrono::milliseconds(5));
            ezio_thread2_.join();
            ezio_thread2_ = thread(&EzioManager::initialEzioComm2, this);

            if (error02_counter_ == 2 && !ezio02_comm_flag_)
            {
                logSys(TrShareEnum::log_err, "[Network] ezioID02 UnConnected");

                auto outMessage = std::make_unique<std_msgs::msg::Int32>(); 
                    outMessage->data = EZIO_02_UNCONNECTED;
                    //Topic write.
                    RCLCPP_INFO(get_logger(), "Topic amr_alarm write: %d", outMessage->data);
                    pub_ezio_alarm_->publish(std::move(outMessage));    
            }

            error02_counter_++;
        }
        else
        {
            error02_counter_ = 0;
        }
    }
    else if (!ezio02_comm_flag_)
    {
        logSys(TrShareEnum::log_err, "[Network] ezioID02 UnConnected");
    }

    if (error03_counter_<3)
    {
        if (!ezio_id03_.reconnectCallback())
        {
            this_thread::sleep_for(chrono::milliseconds(5));
            ezio_thread3_.join();
            ezio_thread3_ = thread(&EzioManager::initialEzioComm3, this);

            if (error03_counter_ == 2 && !ezio03_comm_flag_)
            {
                logSys(TrShareEnum::log_err, "[Network] ezioID03 UnConnected");

                auto outMessage = std::make_unique<std_msgs::msg::Int32>(); 
                    outMessage->data = EZIO_03_UNCONNECTED;
                    //Topic write.
                    RCLCPP_INFO(get_logger(), "Topic amr_alarm write: %d", outMessage->data);
                    pub_ezio_alarm_->publish(std::move(outMessage));    
            }

            error03_counter_++;
        }
        else
        {
            error03_counter_ = 0;
        }
    }
    else if (!ezio03_comm_flag_)
    {
        logSys(TrShareEnum::log_err, "[Network] ezioID03 UnConnected");
    }

    if (error04_counter_<3)
    {
        if (!ezio_id04_.reconnectCallback())
        {
            this_thread::sleep_for(chrono::milliseconds(5));
            ezio_thread4_.join();
            ezio_thread4_ = thread(&EzioManager::initialEzioComm4, this);

            if (error04_counter_ == 2 && !ezio04_comm_flag_)
            {
                logSys(TrShareEnum::log_err, "[Network] ezioID04 UnConnected");

                auto outMessage = std::make_unique<std_msgs::msg::Int32>(); 
                outMessage->data = EZIO_04_UNCONNECTED;
                    //Topic write.
                    RCLCPP_INFO(get_logger(), "Topic amr_alarm write: %d", outMessage->data);
                    pub_ezio_alarm_->publish(std::move(outMessage));    
            }

            error04_counter_++;
        }
        else
        {
            error04_counter_ = 0;
        }
    }
    else if (!ezio04_comm_flag_)
    {
        logSys(TrShareEnum::log_err, "[Network] ezioID04 UnConnected");
    }

    if (error05_counter_<3)
    {
        if (!ezio_id05_.reconnectCallback())
        {
            this_thread::sleep_for(chrono::milliseconds(5));
            ezio_thread5_.join();
            ezio_thread5_ = thread(&EzioManager::initialEzioComm5, this);

            if (error05_counter_ == 2 && !ezio05_comm_flag_)
            {
                logSys(TrShareEnum::log_err, "[Network] ezioID05 UnConnected");

                auto outMessage = std::make_unique<std_msgs::msg::Int32>(); 
                outMessage->data = EZIO_05_UNCONNECTED;
                    //Topic write.
                    RCLCPP_INFO(get_logger(), "Topic amr_alarm write: %d", outMessage->data);
                    pub_ezio_alarm_->publish(std::move(outMessage));    
            }

            error05_counter_++;
        }
        else
        {
            error05_counter_ = 0;
        }
    }
    else if (!ezio05_comm_flag_)
    {
        logSys(TrShareEnum::log_err, "[Network] ezioID05 UnConnected");
    }
}

void EzioManager::initialFileRead()
{
    fileReadSystemEzio();  
}

void EzioManager::fileReadSystemEzio()
{
    
    try
    {
        char cFileData[256];
        char *cptr = NULL;
        int nSeparacounter;

        char c_strBuff[255];
        // strcpy(c_strBuff, m_AbsPath); 
        string home_path = std::getenv("HOME");
	    string log_path = home_path + "/tcon/setting/ezioInfo.txt";
        strcpy(c_strBuff, log_path.c_str());
        // strcat(c_strBuff, "/home/test/tcon/setting/ezioInfo.txt");

        logSys(TrShareEnum::log_seq, "[ezioInfor.txt] loading file");
        

        memset(cFileData, '0', sizeof(cFileData));
        //yh:사용자 폴더까지 넣어야 위치를 확인 할수 있음.
        file_io_module_read_.open(c_strBuff);

        //문서 활성화를 확인한다.
        if(!file_io_module_read_.is_open()){
            RCLCPP_ERROR(get_logger(),"coudl not open ezioInfo.txt fail.."); goto END;   
        }

        //첫번째 데이터 Title 임.
        memset(cFileData, '0', sizeof(cFileData));
        file_io_module_read_.getline(cFileData, sizeof(cFileData)); //데이터 읽어 오기
        cptr = strtok(cFileData,","); //문자열을 "," 이것 기준으로 분리함.
        if( cptr == NULL){
            RCLCPP_ERROR(get_logger(),"FileRead_SystemioModule reading fail.."); goto END;  
        }    

        //다음 줄 가져 오기. 전체 가져올 수량을 확인 한다.
        memset(cFileData, '0', sizeof(cFileData));
        file_io_module_read_.getline(cFileData, sizeof(cFileData)); //데이터 읽어 오기
        cptr = strtok(cFileData,","); //문자열을 "," 이것 기준으로 분리함.
        if( cptr == NULL){
            RCLCPP_ERROR(get_logger(),"FileRead_SystemioModule reading fail.."); goto END;  
        } 

        input_io_module_counter_ = atoi(cptr);
        st_io_mod_info_.ncount = input_io_module_counter_;
        RCLCPP_INFO(get_logger(),"input_io_module_counter_ = %d", input_io_module_counter_ ); 

        //나머지 데이터(라인)을 가져 온다.
        for(int nI = 0; nI <  input_io_module_counter_; nI++)
        {
            memset(cFileData, '0', sizeof(cFileData));
            file_io_module_read_.getline(cFileData, sizeof(cFileData)); //데이터 읽어 오기
            nSeparacounter = 0;
            cptr = strtok(cFileData,","); //문자열을 "," 이것 기준으로 분리함.                      

            while(cptr != NULL) //데이터가 없을때까지 한다.
            {                
                if(nSeparacounter == 0) //ID Index
                    st_io_mod_info_.kData[nI].ncount = atoi(cptr);   					
				else if(nSeparacounter == 2) //Module type input 0 output 1 
                    st_io_mod_info_.kData[nI].nType = atoi(cptr);                  
                else if(nSeparacounter == 3) //Module ID , 0,1,2,3,4,5        
                    st_io_mod_info_.kData[nI].nModuleID = atoi(cptr);					
                else if(nSeparacounter == 4) //Block(32) 1,2,3,4 Block(16) 0(input),1(output)
                    st_io_mod_info_.kData[nI].nBlockID = atoi(cptr);				
                else if( nSeparacounter == 5) //bit No    
                    st_io_mod_info_.kData[nI].nBitID = atoi(cptr);   

				cptr = strtok(NULL, ",");
				nSeparacounter++;
            }

            // RCLCPP_INFO(get_logger(),"Reading return %d, %d, %d, %d, %d", st_io_mod_info_.kData[nI].ncount,
            //                                                           st_io_mod_info_.kData[nI].nType,
            //                                                          st_io_mod_info_.kData[nI].nModuleID,
            //                                                           st_io_mod_info_.kData[nI].nBlockID ,
            //                                                            st_io_mod_info_.kData[nI].nBitID );
                                                              
        }

        RCLCPP_INFO(get_logger(),"FileRead_SystemioModule reading successful");  

        logSys(TrShareEnum::log_seq, "[ezioInfor.txt] reading successful");

        file_io_module_read_.close(); // File exit(close)
        return;     

END: //문제가 있으면 여기서 파일 삭제 하고, 메모리 정리를 한다.
        if(!file_io_module_read_.is_open())
        {
            cptr = NULL; //메모리 리셋
            file_io_module_read_.close(); // File exit(close)
        }
        return;
    }
    catch(const std::exception& e)
    {
            std::cerr << "KKKKK" << e.what() << '\n';
    }   
}

//Io monitring.
void EzioManager::scanEzioCallback()
{ 
    if (ezio01_comm_flag_ && ezio02_comm_flag_ && ezio03_comm_flag_ && ezio04_comm_flag_ && ezio05_comm_flag_)
    {
        DataEzioToPub();
        outWriteToEzio();     
        arrayOutWriteToEzio();
    }
}

void EzioManager::ioServer()
{
     RCLCPP_INFO(get_logger(), "Init createIoEvntSvr Node Start");


    //서버를 생성 한다. Io module에서 메인 시나리오 Packages에서 받는다.
    //IO Module ID, IO-Index, IO-States.
 	auto recieve_manager = [this](const std::shared_ptr<rmw_request_id_t> request_header,
	 						const std::shared_ptr<tc_msgs::srv::IoHeader::Request> request,
                     		const std::shared_ptr<tc_msgs::srv::IoHeader::Response> response) -> void
					{
						(void)request_header;

						response->ret = 1; //응답 플러그

                        selectEzio(request->id, request->state);      

                        srv_event_ = true;  
                        if (request->id>47)
                        {
                            logSys(TrShareEnum::log_seq, "[IO_Services received] IO_pin %d, state %d", request->id,request->state);
                        }
					};

    io_service_ = this->create_service<tc_msgs::srv::IoHeader>("scenario_io_event",recieve_manager);

   RCLCPP_INFO(get_logger(), "Init createIoEvntSvr Node End");    
}


void EzioManager::ioArrayServer()
{
    RCLCPP_INFO(get_logger(), "Init createArryIoEvntSvr Node Start");

    //IO Module ID, IO-Index, IO-States.
 	auto recieve_manager = [this](const std::shared_ptr<rmw_request_id_t> request_header,
	 						const std::shared_ptr<tc_msgs::srv::Io::Request> request,
                     		const std::shared_ptr<tc_msgs::srv::Io::Response> response) -> void
					{
						(void)request_header;

						response->received = true; //응답 플러그

                        int array_size = request->size;

                        int array_id[array_size];
                        int array_state[array_size];

                        //log add
                        std::string array_id_str = "[";
                        std::string array_state_str = "[";

                        for (int i=0; i<array_size; i++)
                        {
                            array_id[i] = request->id[i];
                            array_state[i] = request->state[i];
                            //log add
                            array_id_str += std::to_string(array_id[i]) + " ";
                            array_state_str += std::to_string(array_state[i]) + " ";
                        }
                        //log add
                        array_id_str += "]";
                        array_state_str += "]";

                        arraySelectEzio(array_id, array_state, array_size);      

                        if (array_id[0]>47)
                        {
                            logSys(TrShareEnum::log_seq, "[IO_Array_Services received] IO_array_pin {%s}, IO_array_state {%s}, array_size %d", array_id_str.c_str(),array_state_str.c_str(), array_size);
                        }
					};

    io_array_service_ = this->create_service<tc_msgs::srv::Io>("io_array_event",recieve_manager);
    

    RCLCPP_INFO(get_logger(), "Init createScenarioEvntSvr Node End");    
}
//받은 IO 순번을 모듈 IO로 분리해서 처리한다.
void EzioManager::selectEzio(int id, int state)
{
    try
    {
        iomod_Data kIoCfg;
        if( id >= 300)
        {
            kIoCfg.nModuleID = id;
            kIoCfg.nBlockID = 0;
            kIoCfg.nBitID = 0;
            kIoCfg.state = state;
        }
        else
        {
            kIoCfg.nModuleID = st_io_mod_info_.kData[id].nModuleID;
            kIoCfg.nBlockID = st_io_mod_info_.kData[id].nBlockID;
            kIoCfg.nBitID = st_io_mod_info_.kData[id].nBitID;
            kIoCfg.state = state;
        }

        io_out_setting_.push_back(kIoCfg); //마지막 데이터는 맨 뒤로 보냄.     
    }
    catch(const std::exception& e)
    {
            std::cerr << "KKKKK" << e.what() << '\n';
    } 
}

//output Module 보낼 데이터 확인 하고 보냄.
void EzioManager::outWriteToEzio()
{
    iomod_Data kIoCfg;
    int nModID;
    int nBlockID;
    int nBitNo;
    int state;

    //하나씩 처리 하기 위해 스레드에 넣었다. for문 사용시 
    if(io_out_setting_.size() > 0 )
    {
        kIoCfg = io_out_setting_[0];

        nModID = kIoCfg.nModuleID;
        nBlockID = kIoCfg.nBlockID;
        nBitNo = kIoCfg.nBitID;
        state = kIoCfg.state;

        //RCLCPP_INFO(get_logger(), "OutputModuleSend arry count %d, ID %d", io_out_setting_.size(), nModID);        
        
        switch(nModID)
        {
            case ezioID02:
            {
                ezio_id02_.bitStateToBlockState(ezioID02,nBlockID,nBitNo, state );
                return_set_out_ = ezio_id02_.outEzioSetting();  
                if (!return_set_out_)
                {
                    auto outMessage = std::make_unique<std_msgs::msg::Int32>(); 
                        outMessage->data = EZIO_02_WRITING_FAIL;
                        //Topic write.
                        RCLCPP_INFO(get_logger(), "Topic amr_alarm write: %d", outMessage->data);
                        pub_ezio_alarm_->publish(std::move(outMessage));  
                }
            }  
            break;

            case ezioID03:
            {
                ezio_id03_.bitStateToBlockState(ezioID03,nBlockID,nBitNo, state );
                return_set_out_ = ezio_id03_.outEzioSetting(); 
                if (!return_set_out_)
                {
                    auto outMessage = std::make_unique<std_msgs::msg::Int32>(); 
                        outMessage->data = EZIO_03_WRITING_FAIL;
                        //Topic write.
                        RCLCPP_INFO(get_logger(), "Topic amr_alarm write: %d", outMessage->data);
                        pub_ezio_alarm_->publish(std::move(outMessage));  
                }
            }
            break;

            case ezioID04:
            {
                ezio_id04_.bitStateToBlockState(ezioID04,nBlockID,nBitNo, state );
                return_set_out_ = ezio_id04_.outEzioSetting();
                if (!return_set_out_)
                {
                    auto outMessage = std::make_unique<std_msgs::msg::Int32>(); 
                        outMessage->data = EZIO_04_WRITING_FAIL;
                        //Topic write.
                        RCLCPP_INFO(get_logger(), "Topic amr_alarm write: %d", outMessage->data);
                        pub_ezio_alarm_->publish(std::move(outMessage));  
                }
            }
            break;

            case ezioID05:
            {
                ezio_id05_.bitStateToBlockState(ezioID05,nBlockID,nBitNo, state );
                return_set_out_ = ezio_id05_.outEzioSetting();
                if (!return_set_out_)
                {
                    auto outMessage = std::make_unique<std_msgs::msg::Int32>(); 
                        outMessage->data = EZIO_05_WRITING_FAIL;
                        //Topic write.
                        RCLCPP_INFO(get_logger(), "Topic amr_alarm write: %d", outMessage->data);
                        pub_ezio_alarm_->publish(std::move(outMessage));  
                }
                else
                {
                    logSys(TrShareEnum::log_seq, "[IO-Write Success] ezioID %d, nBlockID %d, nBitNo %d, state %d", nModID+1, nBlockID+1,nBitNo,state);
                    inReceivEzioID05();
                    outReceivEzioID05();
                    //log add
                    std::string array_pin_in = "[";
                    std::string array_pin_out = "[";
                    for (int nI = 0; nI < 8 ; nI++)
                    {
                        if ((in_data_.ID[ezioID05].nBlock01 >> nI & 1) == 1) array_pin_in += "1 ";
                        else array_pin_in += "0 ";

                        if ((out_data_.ID[ezioID05].nBlock02 >> nI & 1) == 1) array_pin_out += "1 ";
                        else array_pin_out += "0 ";
                    }
                    array_pin_in += "]";
                    array_pin_out += "]";
                    logSys(TrShareEnum::log_seq, "[PIO-Status] PIO input {%s}, PIO output {%s}", array_pin_in.c_str(),array_pin_out.c_str());
                }
            }
            break;

        }

        if (return_set_out_) 
        {
            //logSys(TrShareEnum::log_seq, "[Write Success] ezioID %d, nBlockID %d, nBitNo %d, state %d", nModID+1, nBlockID+1,nBitNo,state);
            io_out_setting_.erase(io_out_setting_.begin()); //첫번째 삭제를 한다. 
        }
        else
        {
            logSys(TrShareEnum::log_err, "[Write Failed] ezioID %d, nBlockID %d, nBitNo %d, state %d", nModID+1, nBlockID+1,nBitNo,state);
        }     
    }
    
}

void EzioManager::arraySelectEzio(int  arry_id[], int  array_state[], int size)
{
    try
    {
        iomod_Data io_out_cfg;

        int array_size = size;

        for (int i = 0; i<array_size; i++)
        {
            io_out_cfg.nModuleID = st_io_mod_info_.kData[arry_id[i]].nModuleID;
            io_out_cfg.nBlockID = st_io_mod_info_.kData[arry_id[i]].nBlockID;
            io_out_cfg.nBitID = st_io_mod_info_.kData[arry_id[i]].nBitID;
            io_out_cfg.state = array_state[i];

            io_out_array_setting_.push_back(io_out_cfg); //마지막 데이터는 맨 뒤로 보냄.
        }

        io_out_array_size_.push_back(array_size);

             
    }
    catch(const std::exception& e)
    {
            std::cerr << "KKKKK" << e.what() << '\n';
    } 
}
void EzioManager::arrayOutWriteToEzio()
{
    int size_setting;
    iomod_Data kIoCfg;
    int nModID;
    int nBlockID;
    int nBitNo;
    int state;

    int nModID_list[6];

    //하나씩 처리 하기 위해 스레드에 넣었다. for문 사용시 
    if(io_out_array_setting_.size() > 0 && io_out_array_size_.size() > 0)
    {
        size_setting = io_out_array_size_[0];
        

        //RCLCPP_INFO(get_logger(), "OutputModuleSend arry count %d, ID %d", io_out_setting_.size(), nModID);        
        for (int i = 0; i<size_setting; i++)
        {
            kIoCfg = io_out_array_setting_[i];

            nModID = kIoCfg.nModuleID;
            nBlockID = kIoCfg.nBlockID;
            nBitNo = kIoCfg.nBitID;
            state = kIoCfg.state;

            switch(nModID)
            {
                case ezioID02:
                    ezio_id02_.bitStateToBlockState(ezioID02,nBlockID,nBitNo, state );
                      
                break;

                case ezioID03:
                    ezio_id03_.bitStateToBlockState(ezioID03,nBlockID,nBitNo, state );
                    
                break;

                case ezioID04:
                    ezio_id04_.bitStateToBlockState(ezioID04,nBlockID,nBitNo, state );
                    
                break;

                case ezioID05:
                    ezio_id05_.bitStateToBlockState(ezioID05,nBlockID,nBitNo, state );
                    
                break;

            }
            
        }

        return_set_out_id2_ = ezio_id02_.outEzioSetting(); 
        return_set_out_id3_ = ezio_id03_.outEzioSetting();
        return_set_out_id4_ = ezio_id04_.outEzioSetting();
        return_set_out_id5_ = ezio_id05_.outEzioSetting();

        if (!return_set_out_id2_)
        {
            auto outMessage = std::make_unique<std_msgs::msg::Int32>(); 
                outMessage->data = EZIO_02_WRITING_FAIL;
                //Topic write.
                RCLCPP_INFO(get_logger(), "Topic amr_alarm write: %d", outMessage->data);
                pub_ezio_alarm_->publish(std::move(outMessage));  
        }
        
        if (!return_set_out_id3_)
        {
            auto outMessage = std::make_unique<std_msgs::msg::Int32>(); 
                outMessage->data = EZIO_03_WRITING_FAIL;
                //Topic write.
                RCLCPP_INFO(get_logger(), "Topic amr_alarm write: %d", outMessage->data);
                pub_ezio_alarm_->publish(std::move(outMessage));  
        }
            
        
        if (!return_set_out_id4_)
        {
            auto outMessage = std::make_unique<std_msgs::msg::Int32>(); 
                outMessage->data = EZIO_04_WRITING_FAIL;
                //Topic write.
                RCLCPP_INFO(get_logger(), "Topic amr_alarm write: %d", outMessage->data);
                pub_ezio_alarm_->publish(std::move(outMessage));  
        }

        
        if (!return_set_out_id5_)
        {
            auto outMessage = std::make_unique<std_msgs::msg::Int32>(); 
                outMessage->data = EZIO_05_WRITING_FAIL;
                //Topic write.
                RCLCPP_INFO(get_logger(), "Topic amr_alarm write: %d", outMessage->data);
                pub_ezio_alarm_->publish(std::move(outMessage));  
        }
        else if (return_set_out_id5_ && nModID == ezioID05)
        {
            logSys(TrShareEnum::log_seq, "[IO-Write Success] ezioID %d, nBlockID %d, nBitNo %d, state %d", nModID+1, nBlockID+1,nBitNo,state);
            inReceivEzioID05();
            outReceivEzioID05();
            //log add
            std::string array_pin_in = "[";
            std::string array_pin_out = "[";
            for (int nI = 0; nI < 8 ; nI++)
            {
                if ((in_data_.ID[ezioID05].nBlock01 >> nI & 1) == 1) array_pin_in += "1 ";
                else array_pin_in += "0 ";

                if ((out_data_.ID[ezioID05].nBlock02 >> nI & 1) == 1) array_pin_out += "1 ";
                else array_pin_out += "0 ";
            }
            array_pin_in += "]";
            array_pin_out += "]";
            logSys(TrShareEnum::log_seq, "[PIO-Status] PIO input {%s}, PIO output {%s}", array_pin_in.c_str(),array_pin_out.c_str());
        }
        
        
        if (return_set_out_id2_ && return_set_out_id3_ && return_set_out_id4_ && return_set_out_id5_) 
        {
            for (int i = 0; i<size_setting; i++)
            {
                kIoCfg = io_out_array_setting_[i];

                nModID = kIoCfg.nModuleID;
                nBlockID = kIoCfg.nBlockID;
                nBitNo = kIoCfg.nBitID;
                state = kIoCfg.state;
                //logSys(TrShareEnum::log_seq, "[Write Success] ezioID %d, nBlockID %d, nBitNo %d, state %d", nModID+1, nBlockID+1,nBitNo,state);
            }
            
            io_out_array_setting_.erase(io_out_array_setting_.begin(), io_out_array_setting_.begin()+ size_setting); //첫번째 삭제를 한다.
            io_out_array_size_.erase(io_out_array_size_.begin()); //첫번째 삭제를 한다.
        }
        else
        {
            for (int i = 0; i<size_setting; i++)
            {
                kIoCfg = io_out_array_setting_[i];

                nModID = kIoCfg.nModuleID;
                nBlockID = kIoCfg.nBlockID;
                nBitNo = kIoCfg.nBitID;
                state = kIoCfg.state;
                logSys(TrShareEnum::log_err, "[Write Failed] ezioID %d, nBlockID %d, nBitNo %d, state %d", nModID+1, nBlockID+1,nBitNo,state);
            }
        }     
             
    }
    
}

//Io ezio 01
void EzioManager::inReceivEzioID01()
{
    bool received_flag;

    ezio_id01_.srvCommandSend(0xC0);
    received_flag = ezio_id01_.srvDataReceiv();
    ezio_id01_.returnDataIoStatus(in_data_.ID[ezioID01].nBlock01, 
                                    in_data_.ID[ezioID01].nBlock02, 
                                    in_data_.ID[ezioID01].nBlock03, 
                                    in_data_.ID[ezioID01].nBlock04 );

    if (received_flag && received_flag != ezio01_received_flag_)
    {
        logSys(TrShareEnum::log_seq, "[DATA] ezioID01 receiveIN success");
        ezio01_received_flag_ = received_flag;
    }
    else if (!received_flag && received_flag != ezio01_received_flag_)
    { 
        ezio01_received_flag_ = received_flag;

        auto outMessage = std::make_unique<std_msgs::msg::Int32>(); 
                outMessage->data = EZIO_01_READING_FAIL;
                //Topic write.
                RCLCPP_INFO(get_logger(), "Topic amr_alarm write: %d", outMessage->data);
                pub_ezio_alarm_->publish(std::move(outMessage));  
    }
    if (!received_flag)
    {
        logSys(TrShareEnum::log_err, "[DATA] ezioID01 receiveIN failed");
    }
}
//Io Module 03
void EzioManager::inReceivEzioID03()
{
    bool received_flag;

    ezio_id03_.srvCommandSend(0xC0);
    received_flag = ezio_id03_.srvDataReceiv();
    ezio_id03_.returnDataIoStatus(in_data_.ID[ezioID03].nBlock01, 
                                    in_data_.ID[ezioID03].nBlock02, 
                                    in_data_.ID[ezioID03].nBlock03, 
                                    in_data_.ID[ezioID03].nBlock04 );
    
    if (received_flag && received_flag != ezio03_received_flag_)
    {
        logSys(TrShareEnum::log_seq, "[DATA] ezioID03 receiveIN success");
        ezio03_received_flag_ = received_flag;
    }
    else if (!received_flag && received_flag != ezio03_received_flag_)
    {       
        ezio03_received_flag_ = received_flag;

        auto outMessage = std::make_unique<std_msgs::msg::Int32>(); 
                outMessage->data = EZIO_03_READING_FAIL;
                //Topic write.
                RCLCPP_INFO(get_logger(), "Topic amr_alarm write: %d", outMessage->data);
                pub_ezio_alarm_->publish(std::move(outMessage));  
    }
    if (!received_flag)
    {
        logSys(TrShareEnum::log_err, "[DATA] ezioID03 receiveIN failed");
    }
}
//Io Module 04
void EzioManager::inReceivEzioID04()
{
    bool received_flag;

    ezio_id04_.srvCommandSend(0xC0);
    received_flag = ezio_id04_.srvDataReceiv();
    ezio_id04_.returnDataIoStatus(in_data_.ID[ezioID04].nBlock01, 
                                    in_data_.ID[ezioID04].nBlock02, 
                                    in_data_.ID[ezioID04].nBlock03, 
                                    in_data_.ID[ezioID04].nBlock04 );

    if (received_flag && received_flag != ezio04_received_flag_)
    {
        logSys(TrShareEnum::log_seq, "[DATA] ezioID04 receiveIN success");
        ezio04_received_flag_ = received_flag;
    }
    else if (!received_flag  && received_flag != ezio04_received_flag_)
    {      
        ezio04_received_flag_ = received_flag;

        auto outMessage = std::make_unique<std_msgs::msg::Int32>(); 
                outMessage->data = EZIO_04_READING_FAIL;
                //Topic write.
                RCLCPP_INFO(get_logger(), "Topic amr_alarm write: %d", outMessage->data);
                pub_ezio_alarm_->publish(std::move(outMessage));  
    }
    if (!received_flag)
    {
        logSys(TrShareEnum::log_err, "[DATA] ezioID04 receiveIN failed");
    }
}
//Io Module 05
void EzioManager::inReceivEzioID05()
{
    bool received_flag;

    ezio_id05_.srvCommandSend(0xC0);
    received_flag = ezio_id05_.srvDataReceiv();
    ezio_id05_.returnDataIoStatus(in_data_.ID[ezioID05].nBlock01, 
                                    in_data_.ID[ezioID05].nBlock02, 
                                    in_data_.ID[ezioID05].nBlock03, 
                                    in_data_.ID[ezioID05].nBlock04 );  

    if (received_flag && received_flag != ezio05_received_flag_)
    {
        logSys(TrShareEnum::log_seq, "[DATA] ezioID05 receiveIN success");
        ezio05_received_flag_ = received_flag;
    }
    else if (!received_flag && received_flag != ezio05_received_flag_)
    {    
        ezio05_received_flag_ = received_flag;

        auto outMessage = std::make_unique<std_msgs::msg::Int32>(); 
                outMessage->data = EZIO_05_READING_FAIL;
                //Topic write.
                RCLCPP_INFO(get_logger(), "Topic amr_alarm write: %d", outMessage->data);
                pub_ezio_alarm_->publish(std::move(outMessage));  
    }
    if (!received_flag)
    {
        logSys(TrShareEnum::log_err, "[DATA] ezioID05 receiveIN failed");
    }
}
// OUT Status
//Io Module 02
void EzioManager::outReceivEzioID02()
{
    bool received_flag;

    ezio_id02_.srvCommandSend(0xC5);
    received_flag = ezio_id02_.srvDataReceiv();
    ezio_id02_.returnDataIoStatus(out_data_.ID[ezioID02].nBlock01, 
                                    out_data_.ID[ezioID02].nBlock02, 
                                    out_data_.ID[ezioID02].nBlock03, 
                                    out_data_.ID[ezioID02].nBlock04 );
    
    if (received_flag && received_flag != ezio02_receivedOUT_flag_)
    {
        logSys(TrShareEnum::log_seq, "[DATA] ezioID02 receiveOUT success");
        ezio02_receivedOUT_flag_ = received_flag;
    }
    else if (!received_flag && received_flag != ezio02_receivedOUT_flag_)
    {      
        ezio02_receivedOUT_flag_ = received_flag;

        auto outMessage = std::make_unique<std_msgs::msg::Int32>(); 
                outMessage->data = EZIO_02_READING_FAIL;
                //Topic write.
                RCLCPP_INFO(get_logger(), "Topic amr_alarm write: %d", outMessage->data);
                pub_ezio_alarm_->publish(std::move(outMessage));  
    }
    if (!received_flag)
    {
        logSys(TrShareEnum::log_err, "[DATA] ezioID02 receiveOUT failed");
    }
}
//Io Module 03
void EzioManager::outReceivEzioID03()
{
    bool received_flag;

    ezio_id03_.srvCommandSend(0xC5);
    received_flag = ezio_id03_.srvDataReceiv();
    ezio_id03_.returnDataIoStatus(out_data_.ID[ezioID03].nBlock01, 
                                    out_data_.ID[ezioID03].nBlock02, 
                                    out_data_.ID[ezioID03].nBlock03, 
                                    out_data_.ID[ezioID03].nBlock04 );
    
    if (received_flag && received_flag != ezio03_receivedOUT_flag_)
    {
        logSys(TrShareEnum::log_seq, "[DATA] ezioID03 receiveOUT success");
        ezio03_receivedOUT_flag_ = received_flag;
    }
    else if (!received_flag && received_flag != ezio03_receivedOUT_flag_)
    {     
        ezio03_receivedOUT_flag_ = received_flag;

        auto outMessage = std::make_unique<std_msgs::msg::Int32>(); 
                outMessage->data = EZIO_03_READING_FAIL;
                //Topic write.
                RCLCPP_INFO(get_logger(), "Topic amr_alarm write: %d", outMessage->data);
                pub_ezio_alarm_->publish(std::move(outMessage));  
    }
    if (!received_flag)
    {
        logSys(TrShareEnum::log_err, "[DATA] ezioID03 receiveOUT failed");
    }
}
//Io Module 04
void EzioManager::outReceivEzioID04()
{
    bool received_flag;

    ezio_id04_.srvCommandSend(0xC5);
    received_flag = ezio_id04_.srvDataReceiv();
    ezio_id04_.returnDataIoStatus(out_data_.ID[ezioID04].nBlock01, 
                                    out_data_.ID[ezioID04].nBlock02, 
                                    out_data_.ID[ezioID04].nBlock03, 
                                    out_data_.ID[ezioID04].nBlock04 );
      
    if (received_flag && received_flag != ezio04_receivedOUT_flag_)
    {
        logSys(TrShareEnum::log_seq, "[DATA] ezioID04 receiveOUT success");
        ezio04_receivedOUT_flag_ = received_flag;
    }
    else if (!received_flag && received_flag != ezio04_receivedOUT_flag_)
    {    
        ezio04_receivedOUT_flag_ = received_flag;

        auto outMessage = std::make_unique<std_msgs::msg::Int32>(); 
                outMessage->data = EZIO_04_READING_FAIL;
                //Topic write.
                RCLCPP_INFO(get_logger(), "Topic amr_alarm write: %d", outMessage->data);
                pub_ezio_alarm_->publish(std::move(outMessage));  
    }
    if (!received_flag)
    {
        logSys(TrShareEnum::log_err, "[DATA] ezioID04 receiveOUT failed");
    }
}
//Io Module 05
void EzioManager::outReceivEzioID05()
{
    bool received_flag;

    ezio_id05_.srvCommandSend(0xC5);
    received_flag = ezio_id05_.srvDataReceiv();
    ezio_id05_.returnDataIoStatus(out_data_.ID[ezioID05].nBlock01, 
                                    out_data_.ID[ezioID05].nBlock02, 
                                    out_data_.ID[ezioID05].nBlock03, 
                                    out_data_.ID[ezioID05].nBlock04 );
    
    if (received_flag && received_flag != ezio05_receivedOUT_flag_)
    {
        logSys(TrShareEnum::log_seq, "[DATA] ezioID05 receiveOUT success");
        ezio05_receivedOUT_flag_ = received_flag;
    }
    else if (!received_flag && received_flag != ezio05_receivedOUT_flag_)
    {        
        ezio05_receivedOUT_flag_ = received_flag;

        auto outMessage = std::make_unique<std_msgs::msg::Int32>(); 
                outMessage->data = EZIO_05_READING_FAIL;
                //Topic write.
                RCLCPP_INFO(get_logger(), "Topic amr_alarm write: %d", outMessage->data);
                pub_ezio_alarm_->publish(std::move(outMessage));  
    }
    if (!received_flag)
    {
        logSys(TrShareEnum::log_err, "[DATA] ezioID05 receiveOUT failed");
    }
}

void EzioManager::DataEzioToPub()
{
    inReceivEzioID01(); 
    inReceivEzioID03();
    inReceivEzioID04();
    inReceivEzioID05();

    outReceivEzioID02();
    outReceivEzioID03();
    outReceivEzioID04();
    outReceivEzioID05();

    //메세지 생성.         
    auto outMessage = std::make_unique<tc_msgs::msg::IoModMsg>();         

    //Data shift.
    outMessage->wdword01 = in_data_.ID[ezioID01].nBlock01;
    outMessage->wdword02 = in_data_.ID[ezioID01].nBlock02;
    outMessage->wdword03 = in_data_.ID[ezioID01].nBlock03;
    outMessage->wdword04 = in_data_.ID[ezioID01].nBlock04;
    outMessage->wdword05 = in_data_.ID[ezioID03].nBlock01;
    outMessage->wdword06 = in_data_.ID[ezioID04].nBlock01;
    outMessage->wdword07 = in_data_.ID[ezioID05].nBlock01;
    outMessage->wdword08 = in_data_.ID[ezioID06].nBlock01;

    outMessage->wdword09 = out_data_.ID[ezioID02].nBlock01;
    outMessage->wdword10 = out_data_.ID[ezioID02].nBlock02;
    outMessage->wdword11 = out_data_.ID[ezioID02].nBlock03;
    outMessage->wdword12 = out_data_.ID[ezioID02].nBlock04;
    outMessage->wdword13 = out_data_.ID[ezioID03].nBlock02;
    outMessage->wdword14 = out_data_.ID[ezioID04].nBlock02;
    outMessage->wdword15 = out_data_.ID[ezioID05].nBlock02;
    outMessage->wdword16 = out_data_.ID[ezioID06].nBlock02;
     

    //Topic write.
    pub_ezio_->publish(std::move(outMessage));
}