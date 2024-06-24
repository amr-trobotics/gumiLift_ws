#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//Return data for result;
#define RETURN_NONE 	0
#define RETURN_SUCCESS 	1
#define RETURN_ERR 		2
#define RETURN_BUSY_ON 	3
#define RETURN_CANCEL 	4

// Cart availability
#define CARRIER_NOT_AVAIL		0
#define CARRIER_AVAIL			1
	
// Lidar
#define LIDAR_FRONT 1
#define LIDAR_REAR  2



// Motor Speed for Power Moller Motor
#define NONE_SPEED 		0
#define LOW_SPEED 		1
#define AVERAGE_SPEED 	2
#define HIGH_SPEED 		3

// switch-case final state
#define CASE_ERROR 		99999
#define CASE_SUCCESS 	100000
#define CASE_END 		100001

// Time out
#define ERR_PIO_COMMUNICATION_TIMEOUT       90000 //30s


// Task doing time
#define PIO_SELECT_ON_TIME                  1000 // 1s
#define AMR_MOVING_TIME       				180000 // 180s
#define LIFT_OPERATION_TIME    				60000 

// carrier lot
#define CS_OFF			0
#define BOX		      	1
#define C_BASKET   		2
#define N_BASKET      	3
#define MAGZINE   		4
#define N_BASKET_2EA   	5
#define C_BASKET_2EA	6
#define START_CHARGE 	7
#define END_CHARGE 		8

#define PIO_ID_DOCKING_1	10
#define PIO_ID_DOCKING_2	11
#define PIO_ID_DOCKING_3	12
#define PIO_ID_DOCKING_4	13
#define PIO_ID_AIRSHOWER_1_1	40
#define PIO_ID_AIRSHOWER_1_2	41
#define PIO_ID_AIRSHOWER_2_1	42
#define PIO_ID_AIRSHOWER_2_2	43
#define PIO_ID_CHARGER	60
#define PIO_ID_ELV_1	80
#define PIO_ID_ELV_2	81
#define PIO_RESET	15

#define LIFT_UP	1
#define LIFT_DOWN	2

typedef struct _DockingProcess
{
	int order; 		// order from ACS 1: Loading, 2: Unloading, 3: Charging, 4: Cancel Charging
	int feedback;   // feedback the order = feedback
	int carrier;    // carrier type
	int position;   // based on cylinder sensor, CYL_BASKET_LIMIT, CYL_BOX_LIMIT
	int result; 	// return status: 1->RETURN_SUCCESS, 2->RETURN_ERR
	int error_code; // feedback Error_Code
	   	
}DockingProcess;

typedef struct _MotorMove
{
	int direction;
	int speed;	   	
}MotorMove;

typedef struct _CylinderMode
{
	int mode;   	
}CylinderMode;

enum Error_Code
{
	NO_ERROR					= 0,

	TIMEOUT_LIFT_OPERATION		= 620400,
	TIMEOUT_PIO_DOCKING_COMM	= 620403, // 사용
	TIMEOUT_CHARGING_PROCESS	= 620404, //
	TIMEOUT_CHARGING_OFF_PROCESS= 620405,	

	CARRIER_AVAIL_BUT_REQ_AQUIRE	= 620408,
	PIO_AMR_GO_OFF					= 620409,
	PIO_EQUIP_AND_ACS_ORDER_NOT_MATCH = 620410,
	AQUIRE_METAL_BUT_SENSOR_NOT_DETECTED = 620411,
	CANNOT_RECEIVE_READY = 620412,

	//Add
	VALID_ON_FAIL = 3231,
	U_REQ_L_REQ_FAIL = 3232,
	AS_EQUIPMENT_FAIL = 3233,
	EV_EQUIPMENT_FAIL = 3234,
	STATION_EQUIPMENT_FAIL = 3235,


};

typedef struct _AmrTask
{
	int task;
	int carrier;
	int position; // depend on the carrier, find the position

}AmrTask;


enum Task // matching with order in AmrTask.msg
{
	Unknown 	= 0,
	Docking1 	= 1,
	Docking2 	= 2,
	Docking3 	= 3,
	Docking4    = 4,
	Airshower11 = 5,
	Airshower12 = 6,
	Airshower21 = 7,
	Airshower22 = 8,
	Charging    = 9,
	ChargingOff = 10,
	Elevator1F  = 11,
	Elevator2F  = 12,
	RESET_HPIO  = 13,
	RESET_PIO   = 14,
};

