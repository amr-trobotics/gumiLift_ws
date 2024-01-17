/// @brief all of write log, and time functions.
#include "ezio_manager.h"

namespace fs = std::filesystem;

void EzioManager::loggingBallBack()
{
}
void EzioManager::coordLogging(char* c_strName)
{
}
void EzioManager::tacktimeLogging(char* c_strName, char* c_strData)
{

	std::ofstream ofwriteFile;
	time_t tmcurTime;
	struct tm *ptrpLocal;
    
    tmcurTime = time(NULL);
	ptrpLocal = NULL;
	
	ptrpLocal = localtime(&tmcurTime);
	if(ptrpLocal == NULL)
		return;
	
	string home_path = std::getenv("HOME");
	string log_path = home_path + "/tcon/log/";

	string directory = log_path + to_string(ptrpLocal->tm_year + 1900);
	if(ptrpLocal->tm_mon + 1 < 10)directory = directory + "0";
	directory = directory + to_string(ptrpLocal->tm_mon + 1);
	if(ptrpLocal->tm_mday < 10)directory = directory + "0";
	directory = directory + to_string(ptrpLocal->tm_mday);	
	fs::create_directory(directory); 

	string log_file = directory + "/" + "TackTime_"+ to_string(ptrpLocal->tm_year + 1900);
	if(ptrpLocal->tm_mon + 1 < 10)log_file = log_file + "0";
	log_file = log_file + to_string(ptrpLocal->tm_mon + 1);
	if(ptrpLocal->tm_mday < 10)log_file = log_file + "0";
	log_file = log_file + to_string(ptrpLocal->tm_mday) + ".txt";
	
	ofwriteFile.open(log_file, ios::app);
	
	if(ofwriteFile.is_open())
	{
		ofwriteFile << ptrpLocal->tm_year + 1900 << "-" << ptrpLocal->tm_mon + 1 << "-";
		ofwriteFile << ptrpLocal->tm_mday << " T" << ptrpLocal->tm_hour << " :" << ptrpLocal->tm_min << ":"; 
		ofwriteFile << ptrpLocal->tm_sec << ",";
		ofwriteFile << c_strName << ",";
		ofwriteFile << c_strData;
		ofwriteFile << std::endl;
	}
	
	ofwriteFile.close(); 
	if(ptrpLocal != NULL)
	{
		ptrpLocal = NULL;
		delete ptrpLocal;
	}
}

void EzioManager::log_err(int nErr)
{

	time_t tmcurTime;
	struct tm *ptrpLocal; 

    tmcurTime = time(NULL);
	ptrpLocal = NULL;
	
	ptrpLocal = localtime(&tmcurTime);
	if(ptrpLocal == NULL)
		return;

	logSys(TrShareEnum::log_err,"Err = %d, Description = %s, Div(Warning_0, Heav_1, ByOp_2) = %d ", 
												nErr, st_error_.klist[nErr].strName, st_error_.klist[nErr].nDiv );

	char c_str[255];
    sprintf(c_str,"%04d/%02d/%02d %02d:%02d:%02d ",ptrpLocal->tm_year + 1900, ptrpLocal->tm_mon + 1,ptrpLocal->tm_mday,
													ptrpLocal->tm_hour,ptrpLocal->tm_min,ptrpLocal->tm_sec);
	string strTime(c_str);
	string strErrName(st_error_.klist[nErr].strName);
	int nDiv = st_error_.klist[nErr].nDiv;
	// publishing_Err(strTime, nErr, strErrName, nDiv);

	if(ptrpLocal != NULL)
	{
		ptrpLocal = NULL;
		delete ptrpLocal;
	}	
}
//받은 taskLogging 데이터를 기록한다.
void EzioManager::taskLogging(Tacklist& _Tacklist)
{
	RCLCPP_INFO(this->get_logger(),"nTackNo = %d", _Tacklist.nTackNo);
	RCLCPP_INFO(this->get_logger(),"nTackTypeLen = %d", _Tacklist.nTackTypeLen);
	RCLCPP_INFO(this->get_logger(),"cTackType = %s", _Tacklist.cTackType);
	RCLCPP_INFO(this->get_logger(),"nSlotNo = %d", _Tacklist.nSlotNo);
	RCLCPP_INFO(this->get_logger(),"nMatldLen = %d", _Tacklist.nMatldLen);
	RCLCPP_INFO(this->get_logger(),"cMatld = %s", _Tacklist.cMatld);
	RCLCPP_INFO(this->get_logger(),"lMatDir = %.2f", (double)_Tacklist.fMatDir);
	RCLCPP_INFO(this->get_logger(),"nMatWeight = %d", _Tacklist.nMatWeight);
	RCLCPP_INFO(this->get_logger(),"nTargetPortTypeLen = %d", _Tacklist.nTargetPortTypeLen);
	RCLCPP_INFO(this->get_logger(),"cTargetPortType = %s", _Tacklist.cTargetPortType);
	RCLCPP_INFO(this->get_logger(),"nTargetStageNo = %d", _Tacklist.nTargetStageNo);
	RCLCPP_INFO(this->get_logger(),"nFromPortTypeLen = %d", _Tacklist.nFromPortTypeLen);
	RCLCPP_INFO(this->get_logger(),"cFromPortType = %s", _Tacklist.cFromPortType);
	RCLCPP_INFO(this->get_logger(),"nFromStageNo = %d", _Tacklist.nFromStageNo);
	RCLCPP_INFO(this->get_logger(),"nExitNodeLen = %d", _Tacklist.nExitNodeLen);
	RCLCPP_INFO(this->get_logger(),"cExitNode = %s", _Tacklist.cExitNode);
}

void EzioManager::logSys(int _Type, const char *frm,...)
{
	string strName[] = {"EZIO_SYSTEM","EZIO_COORD","ERR_EZIO","EZIO_TACKTIME","LOG_EZIO"};

	std::ofstream ofwriteFile;

	char tmp[255];
	va_list marker;
	va_start(marker, frm);
	vsprintf(tmp,frm,marker);
	va_end(marker);	

	time_t tmcurTime;
	struct tm *ptrpLocal; 

    tmcurTime = time(NULL);
	ptrpLocal = NULL;
	
	ptrpLocal = localtime(&tmcurTime);
	if(ptrpLocal == NULL)
		return;

	char c_strTime[255];
    sprintf(c_strTime,"%04d/%02d/%02d %02d:%02d:%02d ",ptrpLocal->tm_year + 1900,
													ptrpLocal->tm_mon + 1,
													ptrpLocal->tm_mday,
													ptrpLocal->tm_hour,
													ptrpLocal->tm_min,
													ptrpLocal->tm_sec);
													
	char c_strCombine[4092];
	sprintf(c_strCombine, "%s %s", c_strTime,tmp);

	RCLCPP_INFO(get_logger(),"%s",c_strCombine);//터미널에 Logging 해준다.

	std::string strDirectory = logCreateDirectory(strName[_Type]);
	if( strDirectory != "-1")
	{
		ofwriteFile.open(strDirectory, ios::app);	
		if(ofwriteFile.is_open())
		{
			ofwriteFile << c_strCombine << std::endl;
		}		
		ofwriteFile.close(); 
	}
	
	//메모리 삭제를 한다.
	if(ptrpLocal != NULL)
	{
		ptrpLocal = NULL;
		delete ptrpLocal;
	}

}

std::string EzioManager::logCreateDirectory(string foldername)
{
	struct utsname unameData;
	if (uname(&unameData) == -1) {
        perror("uname");
    }
	char username[256];
    std::strcpy(username, unameData.nodename);

	time_t tmcurTime;
	struct tm *ptrpLocal; 	

    tmcurTime = time(NULL);
	ptrpLocal = NULL;
	
	ptrpLocal = localtime(&tmcurTime);
	if(ptrpLocal == NULL)
		return "-1";
	
	string home_path = std::getenv("HOME");
	string log_path = home_path + "/tcon/log/";

	string directory = log_path + to_string(ptrpLocal->tm_year + 1900);
	
	if(ptrpLocal->tm_mon + 1 < 10)directory = directory + "0";
	directory = directory + to_string(ptrpLocal->tm_mon + 1);
	if(ptrpLocal->tm_mday < 10)directory = directory + "0";
	directory = directory + to_string(ptrpLocal->tm_mday);	
	fs::create_directory(directory); 

	string log_file = directory + "/" +  "[" + to_string(ptrpLocal->tm_year + 1900);
	if(ptrpLocal->tm_mon + 1 < 10)log_file = log_file + "0";
	log_file = log_file + to_string(ptrpLocal->tm_mon + 1);
	if(ptrpLocal->tm_mday < 10)log_file = log_file + "0";
	log_file = log_file + to_string(ptrpLocal->tm_mday) +"]" + "[" + username + "]" + foldername + ".txt";		

	if(ptrpLocal != NULL)
	{
		ptrpLocal = NULL;
		delete ptrpLocal;
	}

	return log_file;
}


//Directory 폴더를 가져와 그 안에 있는 파일 삭제 후 폴더 삭제를 한다.
int EzioManager::rmdirs(const char *path, int force)
{
    DIR *  dir_ptr      = NULL;
    struct dirent *file = NULL;
    struct stat   buf;
    char   filename[1024];

    /* 목록을 읽을 디렉토리명으로 DIR *를 return 받습니다. */
    if((dir_ptr = opendir(path)) == NULL) {
		/* path가 디렉토리가 아니라면 삭제하고 종료합니다. */
		return unlink(path);
    }

    /* 디렉토리의 처음부터 파일 또는 디렉토리명을 순서대로 한개씩 읽습니다. */
    while((file = readdir(dir_ptr)) != NULL) {
        // readdir 읽혀진 파일명 중에 현재 디렉토리를 나타네는 . 도 포함되어 있으므로 
        // 무한 반복에 빠지지 않으려면 파일명이 . 이면 skip 해야 함
        if(strcmp(file->d_name, ".") == 0 || strcmp(file->d_name, "..") == 0) {
             continue;
        }

        sprintf(filename, "%s/%s", path, file->d_name);

        /* 파일의 속성(파일의 유형, 크기, 생성/변경 시간 등을 얻기 위하여 */
        if(lstat(filename, &buf) == -1) {
            continue;
        }

        if(S_ISDIR(buf.st_mode)) { // 검색된 이름의 속성이 디렉토리이면
            /* 검색된 파일이 directory이면 재귀호출로 하위 디렉토리를 다시 검색 */
            if(rmdirs(filename, force) == -1 && !force) {
                return -1;
            }
        } else if(S_ISREG(buf.st_mode) || S_ISLNK(buf.st_mode)) { // 일반파일 또는 symbolic link 이면
            if(unlink(filename) == -1 && !force) {
                return -1;
            }
        }
    }

    /* open된 directory 정보를 close 합니다. */
    closedir(dir_ptr);
    
    return rmdir(path);
}

// struct stat
// {
// 	dev_t st_dev;  // 디바이스 번호
// 	ino_t st_ino;  // inode 번호
// 	mode_t st_mode;  // 모드 (접근권한)
// 	nlink_t st_nlink;  // 하드링크 수
// 	uid_t st_uid;  // 소유자의 사용자 아이디
// 	gid_t st_gid;  // 소유자의 그룹 아이디
// 	dev_t st_rdev;  // 디바이스 아이디 (특수 파일인 경우)
// 	off_t st_size;  // 파일 크기(바이트)
// 	blksize_t st_blksize;  // 블록 크기
// 	blkcnt_t st_blocks;  // 512바이트 블록 갯수
// 	time_t st_atime;  // 최종 접근 시간 (access time)
// 	time_t st_mtime;  // 최종 수정 시간 (modification time)
// 	time_t st_ctime;  // 최종 상태 변경 시간 (change time)
// }
void EzioManager::logRemoveFolder(const char *path)
{

	DIR *  dir_ptr      = NULL;
    struct dirent *file = NULL;
    struct stat   buf;
    char   filename[1024];
	time_t start, end; //현재 날짜와 기준 날짜 비교를 위해 생성.
	double diff;
	int tm_day;

    /* 목록을 읽을 디렉토리명으로 DIR *를 return 받습니다. */
    if((dir_ptr = opendir(path)) == NULL) {
		/* path가 디렉토리가 아니라면 삭제하고 종료합니다. */
		unlink(path);
		return;
    }	

	/* 디렉토리의 처음부터 파일 또는 디렉토리명을 순서대로 한개씩 읽습니다. */
    while((file = readdir(dir_ptr)) != NULL) {
		// readdir 읽혀진 파일명 중에 현재 디렉토리를 나타네는 . 도 포함되어 있으므로 
		// 무한 반복에 빠지지 않으려면 파일명이 . 이면 skip 해야 함
		if(strcmp(file->d_name, ".") == 0 || strcmp(file->d_name, "..") == 0) {
			continue;
		}

		sprintf(filename, "%s/%s", path, file->d_name);	

		/* 파일의 속성(파일의 유형, 크기, 생성/변경 시간 등을 얻기 위하여 */
        if(lstat(filename, &buf) == -1) {
            continue;
        }
		/*현재 시간*/
		start = time(NULL); 
		/*과거 시간*/
		end = buf.st_mtime;
		/*시간 차이*/
		diff = difftime(start, end );
		tm_day = diff / (60 * 60 * 24);
		
		/*Log File 보관은 30일 지정을 한다. 사용자에 의해서 변경을 할까?*/
		if(tm_day > 30)
		{
			if( rmdirs(filename,1) == -1)
			{
				RCLCPP_ERROR(get_logger(),"call logRemoveFolder = Error occurred while deleting file");
				continue;
			}
		}
	}
}


/// Time function
unsigned int EzioManager::getTickCount()
{
    struct timeval tp;
    gettimeofday(&tp,NULL);
    return ((unsigned int)(tp.tv_sec*1000 + tp.tv_usec/1000));
}

//남은 시간을 계산을 한다. - getTickCount() 함수를 받아 게산을 한다.
unsigned long EzioManager::elaspseTime(unsigned int dwStartTickCount)
{
    unsigned int dwTickCount = getTickCount();
             

    if( dwStartTickCount > dwTickCount)        
        return ((dwTickCount - dwStartTickCount) + (unsigned int) 0xffffffff);
           
    return (dwTickCount - dwStartTickCount);    
}
//시간을 셋팅하고, 원하는 시간을 넣어 그 시간을 확인 한다.
bool EzioManager::getTimeOut(unsigned int  dwStartTickCount, int uMilliSecound)
{
    if(elaspseTime(dwStartTickCount) >= uMilliSecound ) return true;
    else return false;
}



