#ifndef _SH_MAIN_
#define _SH_MAIN_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

typedef struct _iomod_Data{
	int ncount; // 전체 수량
	int nType; //INPUT 0. OUTPUT 1
	int nModuleID; //Module No
	int nBlockID; //Block no
	int nBitID; // bit No
	int state; // Status
}iomod_Data;


typedef struct _iomod_info{
	int ncount; // 전체 수량
	iomod_Data kData[200];
}iomod_info;

typedef struct _BitToDec{
	int nBit_State[8];
	int nBit_Value[8]; //진수 값을 넣는다.
	int nSetData;
	int nResetData;
}BitToDec;

typedef struct _bitInfo{
	BitToDec nInfo[4];
}bitInfo;

typedef struct _StepPos{
	double x;
	double y;
	double z;	
}StepPos;

typedef struct _StepData{
	int nType; // 0 Move, 1 angle
	double dblinvel; //속도
	double dbAngvel; //각도
	int nDelayTime; //지연시간
	StepPos kGoalPos; //이동거리
	double dbAngle; //회전 각도.
}StepData;

typedef struct _ScenarioInfo{
	int nScenarioStepCount;
	StepData kStep[300];
}ScenarioInfoInfo;

//xMessageEvent 적용 사용.
typedef struct _IoModuleData{
	int nBlock01;
	int nBlock02;
	int nBlock03;
	int nBlock04;
	int nBlock05;
}IoModuleData;

typedef struct _IoModuleInfo{
	IoModuleData kData[5];
}IoModuleInfo;

typedef struct _SavePosInfo
{
	int index;
	char strWayName[255];
	double pos_x;
	double pos_y;
	double pos_z;
	double ori_x;
	double ori_y;
	double ori_z;
	double ori_w;
	double ori_angle;
	
}SavePosInfo;

typedef struct _WayPointInfo
{
	int nMaxCount;
	SavePosInfo kData[100];

}WayPointInfo;

typedef struct _GoGoalData
{
	SavePosInfo Start; //
	SavePosInfo Goal; //

}GoGoalData;
/*
배열로 처리 되어 있음.
0 D101, G101
1 D102, G102
2 D201, G201
3 D202, G202
*/
typedef struct _GoGoalInfo
{
	int nMaxCount;
	GoGoalData kPos[10]; 

}GoGoalInfo;

typedef struct _Speedlimit
{
	char strName[255];
	int nNodeCount;
	bool bDisInv;
    double dbLimit;
	int nUseDetect_Obstacle;

	
}Speedlimit;

//////////////////////////////////////////////////
typedef struct _Err
{
	char strName[255];
	int nErrCode; //code
	//TrShareEnum::ErrRole
	int nDiv; //Warning = 0(동작 가능하면, 경고 창만 표시 됨), Heavy = 1(동작 불가능), Op = 2(사용자 확인 후 다시 동작 시킴)
	bool bErr; //발생 유무를 확인 한다.
}Err;

typedef struct _ErrorInfo
{
	Err klist[2000];

}ErrorInfo;

typedef struct _Gui_msg
{
	int nDone_no;
	int nCh_no; //PIO NO
	int nCh_done; //completed write to Pio ch with 1 done,
	int noutput_id; //output ID
	bool bSwitch_io; //before write ch, io on / of
	std::string strSetGoalNodeName;
	
}Gui_msg;

//다이얼로그로 데이터 전달
//use DigIo Dialog
typedef struct _IoState
{
	int nIoID[100];
	int state[100];

}IoState;

typedef struct _stGoGoalPlan
{
	bool ReturnMoveComp; //return state after perform the goto_target
	
}stGoGoalPlan;

////////////////////////////
//Login
//20220913
typedef struct _stLoginData
{
	char c_ID[64];
	int nPw;
	int nAuthority; //0 Normal, 1 Super

}stLoginData;

typedef struct _stLogin
{
	int nMaxCount;
	stLoginData Login[10]; // 10 명까지 지원 함.

}stLogin;


#endif /*_SH_IO_*/


