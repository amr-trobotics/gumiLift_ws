#include "ezio_comm05.h"

using namespace std;
/*      Io Module INPUT 32 
        Ezio Model
*/

//생성자
Ezio5::Ezio5()
        : Node("Ezio5")
{
        initialCreate();
        initialPara();
}
//소멸자
Ezio5::~Ezio5()
{
        shutdown(socket_client_fd_, SHUT_WR);
        close(socket_client_fd_);

        RCLCPP_INFO(get_logger(), "~Ezio5");
}
//변수 생성 하기.
void Ezio5::initialCreate()
{
        RCLCPP_INFO(get_logger(), "initialCreate Start");

        RCLCPP_INFO(get_logger(), "initialCreate End");
}
//파라미터 리셋
void Ezio5::initialPara()
{
        send_counter_ = 0;
}

//소켓 통신 설정.
bool Ezio5::initialComm()
{
        RCLCPP_INFO(get_logger(), "initialComm Start");

        // Create socket
        socket_client_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_client_fd_ <= 0) {
                RCLCPP_ERROR(get_logger(), "Socket failed.");
                return false;
        }

        // Set socket to non-blocking mode
        int flags = fcntl(socket_client_fd_, F_GETFL, 0);
        if (fcntl(socket_client_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
                RCLCPP_ERROR(get_logger(), "Failed to set socket to non-blocking mode.");
                return false;
        }

        memset(&st_srv_addr_, 0, sizeof(st_srv_addr_));
        st_srv_addr_.sin_family = AF_INET; // IPv4 주소 인식시킴
        st_srv_addr_.sin_port = htons(2001); // 접속 포트UpdateEzio1

        // 문자열을 프로토콜(Ipv4)등 해당하는 네트워크 데이터 변환(빅엔디언 방식의 이진 데이터)
        if (inet_pton(AF_INET, "192.168.192.12", &st_srv_addr_.sin_addr) <= 0) {
                RCLCPP_ERROR(get_logger(), "inet_pton failed");
                return false;
        }

        // Connect to server
        if (connect(socket_client_fd_, (struct sockaddr*) &st_srv_addr_, sizeof(st_srv_addr_)) < 0) {
                if (errno != EINPROGRESS) {
                RCLCPP_ERROR(get_logger(), "Failed to connect to server.");
                return false;
                }
        } else {
                RCLCPP_INFO(get_logger(), "Connected to server.");
        }

        // Wait for connection to be established
        fd_set fdset;
        FD_ZERO(&fdset);
        FD_SET(socket_client_fd_, &fdset);
        struct timeval tv = {1, 0}; // Set timeout to 5 seconds
        int retval = select(socket_client_fd_ + 1, NULL, &fdset, NULL, &tv);
        if (retval == -1) {
                RCLCPP_ERROR(get_logger(), "Failed to connect to server.");
                return false;
        } else if (retval == 0) {
                RCLCPP_ERROR(get_logger(), "Connection to server timed out.");
                return false;
        } else {
                RCLCPP_INFO(get_logger(), "Connected to server.");
        }

        // Set socket back to blocking mode
        if (fcntl(socket_client_fd_, F_SETFL, flags) < 0) {
                RCLCPP_ERROR(get_logger(), "Failed to set socket back to blocking mode.");
                return false;
        }

        RCLCPP_INFO(get_logger(), "initialComm End");
        return true;
}

bool Ezio5::reconnectCallback()
{
        fd_set write_fds;
        FD_ZERO(&write_fds);
        FD_SET(socket_client_fd_, &write_fds);

        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        int result = select(socket_client_fd_ + 1, NULL, &write_fds, NULL, &timeout);
        if (result == -1) {
                // an error occurred
                // RCLCPP_ERROR(get_logger(), "Failed to connect to server.");  
                return false;
        } else if (result == 0) {
                // the select call timed out, the connection may need to be retried
                // RCLCPP_ERROR(get_logger(), "Connection to server timed out.");
                return false;
        } else {
                // the socket is ready for writing, check the status to see if the connection was successful

                int error = 0;
                socklen_t len = sizeof(error);
                result = getsockopt(socket_client_fd_, SOL_SOCKET, SO_ERROR, &error, &len);
                if (result == -1) {
                // an error occurred
                        // RCLCPP_ERROR(get_logger(), "Failed to connect to server.");  
                        return false;
                } else if (error != 0) {
                // the socket is in an error state, the connection needs to be retried
                        // RCLCPP_INFO(get_logger(), "The socket is in an error state. "); 
                        return false;
                } else {
                // the socket is connected successfully
                        // RCLCPP_INFO(get_logger(), "Connected successfully. ");
                        return true;
                }
        }

}

void Ezio5::initialOutReceive()
{
        srvCommandSend(0xC5);
        srvDataReceiv();
        returnDataIoStatus(stbit_info_.nInfo[0].nSetData,
                                stbit_info_.nInfo[1].nSetData,
                                stbit_info_.nInfo[2].nSetData,
                                stbit_info_.nInfo[3].nSetData);
        
}
//서버에 메세지 보내기.
void Ezio5::srvCommandSend(int task)
{
        unsigned char sBuffer[FD_MAX_SIZE];
        memset(sBuffer, 0, FD_MAX_SIZE);

        //명령을 보낼 때마다 변경을 해주어야 한다.
	send_counter_ += 1;
	if(send_counter_ >= 100) //Overflow 발생을 방지 하기 위해 리셋을 해준다.
	{
		send_counter_ = 1;
	}

        sBuffer[0] = 0xAA;
	sBuffer[1] = 0x03;
	sBuffer[2] = send_counter_;
	sBuffer[3] = 0x00;
	sBuffer[4] = task;

        //데이터 보내기.
        if( send(socket_client_fd_, sBuffer, 5, 0) <= 0 ) //주의) 보내는 길이는 명령어 길이 만큼 보내야 함. 너무 많이 보내면 에러 발생 함.
        {
                RCLCPP_INFO(get_logger(), "Send Failed ... ");
                return;
        } 
}
//서버에서 데이터 리턴 받기.
bool Ezio5::srvDataReceiv()
{
        memset(rev_data_, 0, sizeof(rev_data_));

        //데이터 받기.
        if( recv(socket_client_fd_,rev_data_, FD_MAX_SIZE, 0 ) <= 0 )
        {  
                RCLCPP_INFO(get_logger(), "Receive Failed ... ");	              
                return false;
        }
        
        unsigned int chbyte1 = static_cast<unsigned char>(rev_data_[0]); //Header
        unsigned int chbyte2 = static_cast<unsigned char>(rev_data_[1]); //데이터 길이
        unsigned int chbyte3 = static_cast<unsigned char>(rev_data_[2]); //Counter = 순번
        unsigned int chbyte4 = static_cast<unsigned char>(rev_data_[3]); //0X00
        unsigned int chbyte5 = static_cast<unsigned char>(rev_data_[4]); //명령어
        unsigned int chbyte6 = static_cast<unsigned char>(rev_data_[5]); //데이터01
        unsigned int chbyte7 = static_cast<unsigned char>(rev_data_[6]); //데이터02
        unsigned int chbyte8 = static_cast<unsigned char>(rev_data_[7]); //데이터03
        unsigned int chbyte9 = static_cast<unsigned char>(rev_data_[8]); //데이터03
        unsigned int chbyte10 = static_cast<unsigned char>(rev_data_[9]);
        unsigned int chbyte11 = static_cast<unsigned char>(rev_data_[10]);

        // RCLCPP_INFO(get_logger(), "CommandReceived Id=%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", 
        //                                                5, chbyte1, chbyte2, chbyte3,chbyte4, chbyte5, 
        //                                                chbyte6, chbyte7, chbyte8, chbyte9, chbyte10, chbyte11 );

        wd_data_[0] =	chbyte7;
        wd_data_[1] =	chbyte8;
        wd_data_[2] =	chbyte9;
        wd_data_[3] =	chbyte10; 

        return true; 
}
//xMessageEvent로 데이터 올려 준다.
void Ezio5::returnDataIoStatus(int &data_01, int &data_02, int &data_03, int &data_04)
{
        data_01 = wd_data_[0];
        data_02 = wd_data_[1];
        data_03 = wd_data_[2];
        data_04 = wd_data_[3];
}
//서버에 메세지 보내기.
bool Ezio5::srvCommandSend(int set_data_01, int set_data_02, int set_data_03, int set_data_04,
                                int reset_01, int reset_02 ,int reset_03 ,int reset_04 )
{
        unsigned char sBuffer[FD_MAX_SIZE];
        memset(sBuffer, 0, FD_MAX_SIZE);

        //RCLCPP_INFO(get_logger(), "srvCommandSend, %d, %d, %d, %d, %d, %d, %d, %d", set_data_01, set_data_02, set_data_03, set_data_04 ,
        //                                                                            reset_01,  reset_02,  reset_03, reset_04 );  

        //명령을 보낼 때마다 변경을 해주어야 한다.
	send_counter_ += 1;
	if(send_counter_ >= 100) //Overflow 발생을 방지 하기 위해 리셋을 해준다.
	{
		send_counter_ = 1;
	}

        sBuffer[0] = 0xAA;
	sBuffer[1] = 0x0B;
	sBuffer[2] = send_counter_;
	sBuffer[3] = 0x00; //명령어
	sBuffer[4] = 0xC6; // OUTPUT WRITE 명령어.

        sBuffer[5] = set_data_01; //Set data 01
        sBuffer[6] = set_data_02; //Set data 02
        sBuffer[7] = set_data_03; //Set data 03
        sBuffer[8] = set_data_04; //Set data 04

        sBuffer[9] = reset_01; //Clear data 01
        sBuffer[10] = reset_02; //Clear data 02
        sBuffer[11] = reset_03; //Clear data 03
        sBuffer[12] = reset_04; //Clear data 04

        //데이터 보내기.
        if( send(socket_client_fd_, sBuffer, 13, 0) <= 0 ) //주의) 보내는 길이는 명령어 길이 만큼 보내야 함. 너무 많이 보내면 에러 발생 함.
        {
                RCLCPP_ERROR(get_logger(), "Send Failed ... ");
                return false;
        } 

        RCLCPP_INFO(get_logger(), "Send success ... ");

        if (return_set_out_())
        {
                // check out setting success
                int data_01 = 0;
                int data_02 = 0;
                int data_03 = 0;
                int data_04 = 0;
                std::this_thread::sleep_for(5ms);
                srvCommandSend(0xC5);
                srvDataReceiv();
                returnDataIoStatus(data_01, data_02, data_03, data_04);
                if (data_02 == set_data_02) 
                {
                        RCLCPP_INFO(get_logger(), "data write success ... ");
                        return true;
                }
                else 
                {
                        RCLCPP_INFO(get_logger(), "data write Failed ... ");
                        return false;
                }
        }
        else
        {
                return false;
        }
}
bool Ezio5::return_set_out_()
{
        unsigned int chbyte = 0x11;
        memset(rev_data_, 0, sizeof(rev_data_));

        //데이터 받기.
        if( recv(socket_client_fd_,rev_data_, FD_MAX_SIZE, 0 ) <= 0 )
        {  
                RCLCPP_INFO(get_logger(), "Receive Failed ... ");	              
                return false;
        }
        chbyte = static_cast<unsigned char>(rev_data_[5]); //Header

        if (chbyte==0x00)
        {
                RCLCPP_INFO(get_logger(), "Write output success ... ");
                return true;
        }             
        else
        {
                RCLCPP_INFO(get_logger(), "Write output fail ... ");
                return false;

        }
}
//io output 출력 변경을 한다.
void Ezio5::bitStateToBlockState(int nModID, int nBlockID, int nbitNo, int state)
{ 
        long nBoard, nModule, nActive, nTemp, nPort;
        byte nOutData, nOrgData, nBitData, nLevel;
        byte nReversal; //Reset 할 데이터.

        int nBitNo;

        nBitNo = nbitNo;

        if(state == 1)
         nLevel = (byte)0x01;
        else
         nLevel = (byte)0x00;

        nBitData = (byte)0x01 << nBitNo;
        nOutData = (byte)stbit_info_.nInfo[nBlockID].nSetData;

        nOrgData = (byte)~(nOutData ^ ((byte)0x01 << nBitNo));
        if(state == 1)
                nOrgData |= nBitData;
        else nOrgData &= (byte)(~nBitData);

        nOutData = (byte)~(nOrgData ^ ((byte)0x01 << nBitNo));

        //SetData
        stbit_info_.nInfo[nBlockID].nSetData = (int)nOutData;
        //ResetData
        stbit_info_.nInfo[nBlockID].nResetData = (int)~(nOutData);

        // RCLCPP_INFO(get_logger(), "UpdateIoModuleoutputchange %d, %d, %d, %d, %d, %d ", stbit_info_.nInfo[nBlockID].nSetData, 
        //                                                              stbit_info_.nInfo[nBlockID].nResetData,
        //                                                              nModID, nBlockID, nbitNo, state );   
}
bool Ezio5::outEzioSetting()
{
        return srvCommandSend(stbit_info_.nInfo[0].nSetData,
                        stbit_info_.nInfo[1].nSetData,
                        stbit_info_.nInfo[2].nSetData,
                        stbit_info_.nInfo[3].nSetData,
                        stbit_info_.nInfo[0].nResetData,
                        stbit_info_.nInfo[1].nResetData,
                        stbit_info_.nInfo[2].nResetData,
                        stbit_info_.nInfo[3].nResetData);
}