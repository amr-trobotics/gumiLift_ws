#ifndef _SH_GUI_C
#define _SH_GUI_C

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include "datatype.h"
#include "constants.h"

using namespace std;

//#define FD_MAX_SIZE 20000 //4096000
#define FD_MAX_SIZE 8192 //4096000

////////////////////////////////////////////////////
typedef struct _Pathlist{
	int nFromNodeLen;
	char cFromNode[255]; // Path의 시작 Nodeld - use
	int nToNodeLen;
	char cToNode[255]; // Path의 도착 nodeld - use
	int nPathDirection; //Straigth = 1, ClockWise = 2. CountClockWise = 3

	//Modify
	float fStartMoveAngle; // 해당 Path 이동 시작점에서의 AGV 이동 각도 (degree)  
	float fStartHeadAngle; // 해당 Path 이동 시작점에서의 AGV 전면부 각도 (degree)
	float fEndMoveAngle; // 해당 Path 이동 마지막에서의 AGV 이동 각도 (degree)    	
	float fEndHeadAngle;	 // 해당 Path 이동 마지막에서의 AGV 전면부 각도 (degree)   
	float fRadius; // PathDirection이 ClockWise/CounterClockWise 일 때 곡률    

	
	
	
	float fAngleStart; //Agv가 해당 Path에 진입시 유지해야 하는 각도(degree)
	float fAngleEend;  //Agv가 해당 Path의 to Node에 도착 시 유지해야 하는 각도
	float fToNodeAngel; //Agv가 toNode에 도착 시 유지 해야 하는 각도
	long lTimeStart; //Path 진입시간(ms) - use
	long lTimeEnd; //Path 완료 시간(ms) - use
	long lStartBoadyTurn; // body Turn 시작 시간(ms)
	long lStartWait; //대기 시작 시간(ms)
	long lStartAccel; //가속 시작 시간(ms)
	long lStartMove1; //직선 구간 진입 시간(ms)
	long lStartMoveR; //곡선 구간 진입 시간(ms)
	long lStartEdgeWait; //EdgeWait 시작 시간(ms)
	long lEdgeWaitEnd; //EdgeWait 종료 시간(ms)
	long lStartMove2; //지선 구간 진입 시간(ms)
	long lStartBreak; //감속 시작 시간(ms)
	float fDistAccel; //가속 구간 거리(mm)
	float fDistMove1; //직선 구간 1 이동 거리(mm)
	float fDistMoveR; //곡선 구간 이동 거리(mm)
	float fDistMove2; //직선 구간 2 이동 거리(mm)
	float fMaxVelocity; //Path 이동 시 최대 이동 속도 (mm/s)
}Pathlist;

typedef struct _tacklist{
	int nTackNo; //수행할 세부 작업의 순번.
	int nTackTypeLen;
	char cTackType[64]; //작업 내용
	int nSlotNo; //Agv 반송물이 적재된 Slot 번호 0 ~
	int nMatldLen; //길이 지정
	char cMatld[32]; //Task를 수행할 반송물의 식별 ID,
	float fMatDir; //반송물의 방향
	int nMatWeight; //0 empty, 1 loaded
	int nTargetPortTypeLen;
	char cTargetPortType[32]; // "LD=반송물을 투입 할 PORT", "ULD=반송물을 AMR로 적재할 port"
	int nTargetStageNo; //
	int nFromPortTypeLen;
	char cFromPortType[32];
	int nFromStageNo;
	int nExitNodeLen;
	char cExitNode[32]; //ASE_EXIT, ASX_EXIT, EL_EXIT, UNDOCKING 명령일 때, EXIST로 이동해야 할 Node 정보


//Modify
	int nTargetMapLen;
	char cTargetMap[64]; //EL_EXIT 명령일 때, Elevator를 통해 이동해야 할 층의 MapName

	int nFromNodeLen;
	char cFromNode[255]; // 해당 Task에서 Node 변경시 사용
	int nToNodeLen;
	char cToNode[255]; // 해당 Task에서 Node 변경시 사용

}Tacklist;

typedef struct _MoveWorkInfo{
	int nJobIDLen;
	char JobID[255]; //ACS로 부터 하들 받아 진행 중인 JOB ID
	int nChangeRoute; // 이전 path 정보를 Clear 해야 하는 경우 0: none, 1 remove previously requested path list
	int nFinalGoalLen;
	char cFinalGoal[255]; //최종 목적지 Node
	int nPathCount; //Path 개수
	Pathlist kPathlist[50];
    //--------------------------- path list
	int nOperationTarget; //0 None, 1 Stocker port, 2 설비 PORT, 3 자동 Door, 4 충전, 5 Elevater, 6 Airshower.
	int nTargetNameLen;
	char cTargetName[255]; //작업자가 AGV의 목적지를 쉽게 인지 할 수 있도록 Dispaly 할 text
	int taskCount; //도착지지에서 수행할  Task 개수
	//---------------------------- task list.
	Tacklist kTacklist[50];	
}MoveWorkInfo;

///////////////////////////////////////////////////////////

typedef struct _MoveWorkCancel
{
	int nJobIDLen;
	char cJobID[255]; //작업 취소를 하는 이 명령의
	int nJobIdToCancelLen;
	char cJobIdToCancel[255]; //Cancel 처리 할 JobID
	int nCancelOption; // 0 취소 요청, 1 진행 중인 작업 무조건 취소.

}MoveWorkCancel;

typedef struct _MoveWorkPause
{
	int nAtLen;
	char cAt[255]; //empty 현재 위치 Nodeld : 지정된 Node 이동 후 정지.

}MoveWorkPause;

typedef struct _MoveWorkScan
{
	char cJobID[255]; //이 명령의 JobID

}MoveWorkScan;

typedef struct _MoveWorkAlarm
{
	int Action; //0 : off AGV 의 Alarm 표시 해제 or 1 : on AGV 의 Alarm 표시
	int nMsgLen;
	char Msg[255];

}MoveWorkAlarm;

typedef struct _MapNamelist
{
	int nLen;
	char c_name[64];
	int MapWidth; //mm단위로 환산한 실제 Map Width
	int MapHeight; //mm단위로 환산한 실제 Map Height

//Modify
	char cMapVer[255]; // MapVersion
	int nMapVerLen;

}MapNamelist;

typedef struct _CheckMapInfo
{
	int nMaxMapCount;
	int nMapVerLen;
	char cMapVer[255]; //AGV가 Salam등으로 작성한 Map 버전명이며, ACS에서 현재 사용하고 있는 값
	int nMetaDataVerLen;
	char cMetaDataVer[255]; //ACS에서 Map 편집 툴을 통해 추가한 Map Meta Data 버전명이며, ACS에서 현재 사용하고 있는 값.
	int nMapCount; //Agv 가 가지고 있는 map 개수
	MapNamelist  kMaplist[16]; //Map name string array

	//Modify
	double dMapOriginX01;
	double dMapOriginY01;	
	double dMapOriginX02;
	double dMapOriginY02;

}CheckMapInfo;

typedef struct _MoveWorkStopCharge
{
	int nAction; //받은 명령어,
    int nJobIDLen;
	char cJobID[255]; //이 명령의 JobID

}MoveWorkStopCharge;

/////////////////////////////////////////////////
typedef struct _MapItem
{
	int idLen;
	char strID[255]; //ID of Item
	double x; //item의 죄표
	double y; //item의 죄표
	float fAngle; // not use
	int nType; //item 0 waypoint node , 1 Docking Node, 2 charge Node, 3 Elevator Node, 4 AirShower Node
	int nMapNameLen;
	char strMapName[255]; //Item이 속한 Map의 Name;

}MapItem;

typedef struct _UpdateMapInfo
{
	int nMapNameLen;
	char cMapName[255]; //Map Name
	int nMapIntemCount; //item 개수
	_MapItem kMapItem[255];
	float fRTurnRadious; //Map에서 공통적으로 사용할 R-Turn Radious
	int nMapDataSize;
	int nMapDataArry[255];
		
	double OriginX;
	double OriginY;
	int MapWidth;
	int MapHeight;
	
}UpdateMapInfo;

///////////////////////////////////////////////
typedef struct _FetchMapInfo
{
	int nMapNameLen;
	char cMapName[255];
	int nReceiveOk; // 1 Accepted, 0 Reason
	int nFailReason;
	int nMapType; //0 png, 1 bmp, 2 jpg
	int nMapItemCount; //Map DAta
	_MapItem kMapItem[255];
	int nMapWidth; // mm 단위로 환산한 실제 Map width
	int nMapHeight; // mm 단위로 환산한 실제 Map Height
	int nMapDataSize;
	int nMapData[255];

}FetchMapInfo;

///////////////////////////////////////
typedef struct _CarrierInfo
{
	int slotID;
	int nMatIDLen;
	char strMatID[255];
	int nMatWeight; //0 empty(공 매거진), 1 loaded(실 매거진)
	long lMatDir; //반송물의 방향.
}CarrierInfo;

typedef struct _InitReport
{
	long lXpos; //(mm)
	long lYpos; //(mm)
	int nMapNameLen;
	char strMapName[255]; //AGV 가 위치한 현재의 MapName
	long lAngle; //AGV 의 각도 (degree)
	long lBattery; //Battery Percent (%)
	int nCurrentNodeLen;
	char strCurrentNode[255]; //현재 위치한 Node ( 알 수 없다면 empty 로 전송 )
	long lLiftState; //Lift 상태 값 . (0 : Lift Down, 1: Lift Up)
	int nNumOfCarrier; //장착된 Carrier 의 개수 ( 알 수 없다면 0 으로 전송 )
	_CarrierInfo kCarrierlist[255];

}InitReport;

////////////////////////////////////////
typedef struct _StatusReport
{
	double lXpos; //현재 위치
	double lYpos; //현재 위치
	int nMapNameLen;
	string strMapName;
	double lAngle;
	double lVelocity; //Node 도착 또는 현재 위치 전송 시점의 이동 속도.
	double lBattery;
	int nCurrentNodeLen;
	string strCurrentNode; //AGV가 가장 최근에 지난간 Node
	int nNextNodeLen;
	string strNextNode; //AGV가 현재 이동하려고 하는 Node.

}StatusReport;

////////////////////////////////////
typedef struct _MoveCompletReport
{
	int nJobIDLen;
	char strJobID[255];   //Cancel 된 경우 여기에 Cancel JobID 넣는다.
	int nOrgJobIDLen;
	char strOrgJobID[255]; //기존 정보를 넣는다.
	int nCompletedType; //0 N = Normal, 정상적으로 수행 완료 됨
	                    //1 F = 현장에서 AGV를 통해 강제 조치로 해당 Job 종료 처리.
						//2 C = ACS로 부터 moveWorkCancel을 받아서 작업 중단 종료.
						//3 E = Error, 작업이 실패하여 정상적으로 완료되지 못 함.
}MoveCompletReport;

typedef struct _FailureReport
{
	int nJobIDLen;
	string strJobID;
	int nRequiredAction;
							//failure 와 필요한 조치 수준
							//0 : Info = 일시 중단 (Timeout 등 ), 작업자 조치가 필요 없는 경우
							//1 : Alarm = 현장 알람을 울리고 , 작업자의 조치를 기다림
							//2 : Reject = 현장에서 취소 조치 완료 , 작업 강제 취소 요청
	long lStopReason;

}FailureReport;

///////////////////////////////
typedef struct _StopReport
{
	int nReason; 

}StopReport;

/////////////////////////////////////////////
typedef struct _ContinueReport
{
	int nJobIDLen;
	string strJobID; 
	
}ContinueReport;


///////////////////////////////
typedef struct _TaskReport
{
	int nJobIDLen;
	string strJobID; 
	int taskNo; //완료된 Task No. 

}TaskReport;

////////////////////////////////
//AGV는 ACS로 지정된 T1 = 100ms 주리고 keepAlive 메시지를 보내야 한다.
typedef struct _KeepAliveReport
{
	int nJobIDLen;
	string strJobID; 
	double lXpos; //현재 위치
	double lYpos; //현재 위치
	int nMapNameLen;
	string strMapName;
	double lAngle;
	int nCurrentNodeLen;
	string strCurrentNode; //AGV가 가장 최근에 지난간 Node
	string strNextNode;
	long lBattery;
	int nliftState; //0 = lift Down, 1 lift up
	int nAgvState;
	int nAutoMode; //0 = manual mode, 1 = auto mode


}KeepAliveReport;

typedef struct _UseVal
{
	int nUse;
	double dbData;
	long lData;
	int nData;
	string strData;
}UseVal;

typedef struct _SysData
{
	char ItmeName[255]; //구분
	char NodeName[255]; //노드
	int nNodeIndex; //순번을 가지고 있음.(여기로 비교 하여 정방향 역방향 정한다.
	int nPIOID; //Pio Slav ID

	//int nGoalbeUse; //사용 유무
	//double dbGoalbe;  //최종 목적지에 도착 후 수행 거리

	//int nGoalPioUse; //사용 유무
	//double dbGoalPio; //최종 목적지에서 Aruco와 거리

	//int nGoalBackUse; //사용 유무
	//double dbGoalBack; // 충전 또는 Docking 후 뒤로 이동시 거리.
	//int nGoalForward;
	//double dbGoalForward;
	//double dbFinalAngle;
	
	_UseVal Aruco_befor; //최종 목적지에 도착 후 수행 거리
	_UseVal Aruco_Middle; //최종 목적지에서 Aruco와 거리
	_UseVal Back_linear;
	_UseVal Forward_linear;
	_UseVal Forward_Angle;
	_UseVal Final_Angle;
	_UseVal PosSpeedLimit;
	_UseVal NogSpeedLimit;
	_UseVal Detect_Obstacle;

}SysData;

typedef struct _SysBms
{
	int nSoc; //충전율
	int state; //상태
	int nSoh; //사용량
	float fCur;
	int nChargeRate;
	int nChargeComp; //
}SysBms;

typedef struct _SysSpd
{
	_UseVal flinear;
	_UseVal fAngular;
}SysSpd;

//20220707 by Yang
typedef struct _SysRider
{
	_UseVal Dis; //측정 거리
	_UseVal Pos; //CW(+) range
	_UseVal Neg; //CCW(-) range	

}SysRider;

typedef struct _ErrData
{
	string strHappen_Time;
	int nErrCode;
	string strDesc;
}ErrData;

typedef struct _RobotState
{
	int nSeqCount;
	int nRustate; //0 None, 1 run , 2 Idle, 3 down, 4 initial, 5, Stop, 6 Pause, 
	int nCancelState; //0 None, 1 Canceling, 2 Completed,
	//bool bMode; //Auto true, Manual false;
	bool bAutoMode; //20221103
	bool bPause; // 무엇에 의해서 정지 했을 경우 상태 표시를 한다.
	bool bStop;
	bool bEvDoorOpen; //Door open 상태를 표시 한다. - Rider에 의해서 결정한다.
	int nLift_height;
	int nCarrierExist; //0 empty, 1 Ok, 2 ng
	bool bObstacle_Lv01;
	bool bObstacle_Lv02;
	bool bObstacle_Lv03;
	bool bObstacle_side02;
	bool bObstacle_side03;
	bool bObstacle_side04;
	bool bObstacle_side05;

	double dbRotationSpd;
	int nleftAfterTime; //Lift 후 후진시 Delay Time
	
	SysSpd kCurSpd; //현재 속도 표시
	_SysRider stRider[10]; //사방 및 안전거리 문 감지. 20220707 by Yang.

	int nBumperFrontBypass;
	int nBumperRearBypass;
	int nBumperFrontState;
	int nBumperRearState;
	int nEMO1State;
	int nEMO2State;
	int nEMO3State;
	int nEMO4State;

	int nUseDetectObstacle; // 전방에 감지 영역을 확인 하고, 속도 조절을 할 것인가? 사용 유무
	int nAcs_Connected; //연결 상태 (1: Connected, 0 Disconnected)
	int nAcs_UseKeepAlive; //AGV는 ACS로 지정된 T1(default = 100 ms) 주기로 KeepAlive 메시지

	bool alarm_popup;
	int alarm_action; //0 : off AGV의 Alarm 표시 해제, 1 : on AGV의 Alarm 표시
	string alarm_msg; //#Alarm On 시 같이 표시할 추가 메시지
	int nAgv_State;

	string current_node;
	string next_node;

	bool bObstacleRelase;

}RobotState;

typedef struct _TimeoutData
{
	int nTm;

}TimeoutData;

typedef struct _TimeoutInfo
{
	TimeoutData stAirShow[10];
	TimeoutData stElevator[10];
	TimeoutData stStation[10];
	TimeoutData stCharge[10];

}TimeoutInfo;

typedef struct _AMR_Report
   {
      unsigned char amr_msgtype;
      string jobid;
      string org_jobid;

      double xpos; //(mm)
      double ypos; //(mm)
      string map_name; //AGV 가 위치한 현재의 MapName
      double angle; //AGV 의 각도 (degree)
      double velocity;
      double battery; //Battery Percent (%)
      string current_node; //현재 위치한 Node ( 알 수 없다면 empty 로 전송 )
      string next_node;
      int lift_state; //Lift 상태 값 . (0 : Lift Down, 1: Lift Up)
      int num_carrier; //장착된 Carrier 의 개수 ( 알 수 없다면 0 으로 전송 )
      int slotid[255];
      string matid[255];
      int mat_weight[255]; //0 empty(공 매거진), 1 loaded(실 매거진)
      double mat_dir[255]; //반송물의 방향 (degree) double

      int complete_type;
      int required_action;
      long stop_reason;
      int agv_state;
      int task_no;
	  int auto_mode;
	  //CheckMap
	  string mapver;//AGV가 SLAM등으로 작성한 Map 버전명
	  string metadataver;//ACS에서 Map 편집 툴을 통해 추가한 Map Meta Data 버전명
	  string mapname[16]; //Map name string array

   }AMR_Report;


// #define AcsActionCommandAt_None 0
// #define AcsActionCommandAt_Wait 1
// #define AcsActionCommandAt_Ing 2
// #define AcsActionCommandAt_Comp 3
typedef struct _AcsActionCommandAt
{
	bool bIm; //immediately
	int nActionPlay; //AcsActionCommandAt
	char NodeName[65];
	char JobId[255]; //작업 취소를 하는 이 명령의 JobId
	char JobIdToCancel[255]; //Cancel 처리할 JobId

}AcsActionCommandAt;

typedef struct _Systeminfo
{
	int nVer;
	int nSeqNo; //Seq필드는 Command, Event 메시지는 신규 메시지 발행 시마다 1씩 증가시킨다.
	int nAgvState; //Appendix.B에서 정의한 AGV의 상태 변경 시 ACS로 보고하는 이벤트

	//last goal
	char strlastdestination[64]; //수행 마지막 위치를 저장을 한다.
	int nlastdestination; //수행 마지막 위치를 저장을 한다. Index
	int nFloorNo; //층을 업데이트 한다.
	double dbCurrentSpeeed; //현재 사용하고 있는 속도 저장을 한다. WayPoint Event 받아 저장
	// char strMapName01[255]; //1층 맵 명
	// char strMapName02[255]; //2층 맵 명
	// char strMapVer[255]; //AGV가 SLAM등으로 작성한 Map 버전명이며
	// char strMetaDataVer[255];

	bool bSystemErrorHappen;
	bool bOkPioIDCreate; //정상 생성 됨.	
	int nSysDataMaxCount;
	bool bReverse; // 역방향(Waypoint 방향을 이야기함.)

	//20221027
	double lfW208_Angle;
	double lfW210_Angle;
	double lfW208_Angle_Back;
	double lfW210_Angle_Back;
	//20221027
	SavePosInfo stw208_cd;
	SavePosInfo stw210_cd;

	_KeepAliveReport  keep;
	_SysData stSysData[100];
	_SysBms  stBms; //Bms 정보.
	_SavePosInfo stlastcoord;
	_RobotState stRbtState;
	_TimeoutInfo stTimeout;
	_AMR_Report kEvenReport;
	_AcsActionCommandAt kCacnelbyOp;
	_AcsActionCommandAt kCancel;
	_AcsActionCommandAt kPause;
	_AcsActionCommandAt kStopCharge;	
	_CheckMapInfo kMapInfo; //이노텍 프로젝트 2층 0->Frist, 1->Second 

}Systeminfo;

typedef struct _InitReportResp
{
	int resultCode;
	int KeepAlive_Period;
	int KeepAlive_Count;
}InitReportResp;

//ACSmsg
typedef struct _ACS_recv
{
	unsigned char     acs_msgtype;
	string            jobid; 
	string            jobid_cancel;    
	int               ping_count;     
	long              current_time; 
	string            pause_at;        
	int               alarm_action; 
	string            alarm_msg;
	string            mapver; 
	string            meta_dataver;
	int               result_code;         
	int               keep_alive_period;  
	int               keep_alive_count; 
	int 			  host_connected; //연결 상태 확인.
}ACS_recv;

//응답 받기.
//ACS → AGV Response List
//각각의 event 에 대한 ACS의 Response 메시지는 하기와 같다..
typedef struct _Resp_recv
{
	bool bInitReportResp;
	bool bStatusReportResp;
	bool bMoveCompletedResp;
	bool bFailReportResp;
	bool bContinueReportResp;
	bool bStateChangedResp;
	bool bTskReportResp;
	bool bKeepAliveResp;

}Res_recv;



typedef struct _HostRecvData{
	int nStart;
	int nEnd;
	char strdata[FD_MAX_SIZE];
}HostRecvData;

typedef struct _HostRecvDataInfo
{
	int nMaxNm;
	_HostRecvData kData[32];

}HostRecvDataInfo;


       

   

#endif /*_SH_HOST_C*/


