#ifndef _YH_
#define _YH_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//**********EZIO IN************//
//EZIOCH01 define I32N-T
#define IN_MC1_ON 0 // NC - 0 Alarm - Charger MC
#define IN_MC2_ON 1 // NC - 0 Alarm T-Driver Motor
#define IN_REAR_DOOR1 2 // Stop
#define IN_REAR_DOOR2 3 // Stop
#define IN_TDRIVE_ALARM 4 // TDRIVER ALARM
#define IN_TDRIVE_ON 5 // TDRIVER TURN ON
#define IN_6 6  //TASK
#define IN_STD01_ALARM_OUT 7 //STOPPER ALARM 
#define IN_POWERMOLLER1_ERROR 8 // POWER moller Error TASK
#define IN_POWERMOLLER2_ERROR 9 // POWER moller Error TASK
#define IN_SPARE_SW 10 // MC, ON when charge, OFF when stop charge
#define IN_RESET_SW 11 // Reset
#define IN_BUZZER_STOP_SW 12 // Sound Off
#define IN_BREAK_RELEASE_SW 13 //Push lock  
#define IN_SAFETY_RELEASE_SW 14 //Push lock
#define IN_CV_GUIDE_RELEASE_SW 15 // CYLINDER Backward TASK
#define IN_FRONT_CLAMP_DETECTION1 16 // Contact Sensor Detect TASK
#define IN_FRONT_CLAMP_DETECTION2 17 // Contact Sensor Detect TASK
#define IN_REAR_CLAMP_DETECTION1 18 //Contact Sensor Detect TASK
#define IN_REAR_CLAMP_DETECTION2 19 //Contact Sensor Detect TASK
#define IN_STOPPER_UP 20 // STOPPER UP SENSOR TASK
#define IN_STOPPER_DOWN 21 // STOPPER DOWN SENSOR TASK
#define IN_BOX_DETECTION1 22 // TASK
#define IN_BOX_DETECTION2 23 // TASK
#define IN_BOX_DETECTION3 24 // TASK
#define IN_FRONT_DOOR1 25 // front door 
#define IN_FRONT_DOOR2 26 // front door
#define IN_BATTERY_DOOR 27 // Deleted , Don't use
#define IN_REAR_CYLINDER_BWD 28 // don't use
#define IN_REAR_CYLINDER_BOX_LIMIT 29 // don't use 
#define IN_REAR_CYLINDER_BASKET_LIMIT 30 // don't use
#define IN_REAR_CYLINDER_FWD_LIMIT 3100 //don't use    (need to delete)
#define IN_PIO_GO 31 //PIO GO
/*
    #define IN_REAR_RWD_DETECTION 28 // TASK
    #define IN_REAR_BOX_DETECTION 29 // TASK 
    #define IN_REAR_BASKET_DETECTION 30 // TASK
    #define IN_REAR_FWD_DETECTION 31 //TASK
*/

//EZIOCH03 define I8O8P-T
#define IN_LIDAR1_WARNING 32 // lidar1 warning zone
#define IN_PALLET_DETECTION_1 33 // BOX IN
#define IN_PALLET_DETECTION_2 34 // BASKET IN
#define IN_LIDAR1_IN04 35 // Don't use
#define IN_LIDAR1_PROTECTIVE 36 // lidar1 protective zone
#define IN_SAFETY_ERROR 37 // Safety PLC ErrorSafety, PLC Device Error Status
#define IN_SAFETY_RESET 38 // Safety PLC Reset, Reset Button
#define IN_BUMP12_NORMAL 39 // 4 Bumpers Only use this one
//EZIOCH04 define I8O8P-T
#define IN_LIDAR2_WARNING 40 // Don't use
#define LIDAR1_ERROR 41 // Don't use
#define LIDAR1_CLEANING 42 // Don't use
#define LIDAR2_ERROR 43 // Don't use
#define IN_LIDAR2_PROTECTIVE 44

#define IN_MANUAL_KEY 45 // Manual
#define IN_AUTO_KEY 46 // Auto
//EZIOCH05 define I8O8P-T
#define LIDAR2_CLEANING 47
#define IN_PIO_OUT01_LREQ 48
#define IN_PIO_OUT02_UREQ 49
#define IN_PIO_OUT03 50
#define IN_PIO_OUT04_READY 51
#define IN_PIO_OUT05 52
#define IN_PIO_OUT06 53
#define IN_PIO_OUT07 54
#define IN_PIO_OUT08_ESTOP 55
//EZIOCH06 define I8O8P-T
#define IN_FRONT_CYLINDER_BWD 56 // TASK
#define IN_FRONT_CYLINDER_BOX_LIMIT 57 // TASK 
#define IN_FRONT_CYLINDER_BASKET_LIMIT 58 // TASK
#define IN_FRONT_CYLINDER_FWD_LIMIT 59 //Servo Driver T-Driver, Don't use
#define IN_FRONT_LEFT_EMS 60  //EMS
#define IN_FRONT_RIGHT_EMS 61 //EMS
#define IN_REAR_LEFT_EMS 62   //EMS
#define IN_REAR_RIGHT_EMS 63  //EMS

//**********EZIO OUT************//
//EZIOCH02 define O32N-T
#define OUT_STD01_ALARM_RESET 0 // Stopper Alarm Reset
#define OUT_PIO_ONOFF 1 
#define OUT_AIRLINK_ONOFF 2
#define OUT_DISTANCE_ONOFF 3
#define OUT_PIO_SELECT 4  // PIO select updated
#define OUT_BUZZER_CH2 5
#define OUT_BUZZER_CH3 6
#define OUT_BUZZER_CH4 7
#define OUT_BUZZER_CH5 8
#define OUT_BUZZER_STOP 9
#define OUT_LOWER_REAR_LEFT_G 10
#define OUT_LOWER_REAR_LEFT_R 11
#define OUT_LOWER_REAR_LEFT_B 12
#define OUT_LOWER_REAR_RIGHT_G 13
#define OUT_LOWER_REAR_RIGHT_R 14
#define OUT_LOWER_REAR_RIGHT_B 15
#define OUT_STD01_CWCCW_STOP 16
#define OUT_STD01_START 17
#define OUT_STD01_STOP 18
#define OUT_POWERMOLLER1_REVERSE 19
#define OUT_POWERMOLLER1_RUNA 20
#define OUT_POWERMOLLER1_RUNB 21
#define OUT_POWERMOLLER2_REVERSE 22
#define OUT_POWERMOLLER2_RUNA 23
#define OUT_POWERMOLLER2_RUNB 24
#define OUT_UPPER_LED_G 25
#define OUT_UPPER_LED_R 26
#define OUT_UPPER_LED_B 27
#define OUT_FRONT_CYLINDER_FORWARD 28
#define OUT_FRONT_CYLINDER_BACKWARD 29
#define OUT_REAR_CYLINDER_FORWARD 30
#define OUT_REAR_CYLINDER_BACKWARD 31
//EZIOCH03 define I8O8P-T
#define OUT_LIDAR1_C1 32
#define OUT_LIDAR1_C2 33
#define OUT_LIDAR1_D1 34
#define OUT_LIDAR1_D2 35
#define OUT_LIDAR1_E1 36
#define OUT_LIDAR1_E2 37
#define OUT_LIDAR1_RESTART 38
#define OUT_LIDAR1_OUT08 39
//EZIOCH04 define I8O8P-T
#define OUT_LIDAR2_C1 40
#define OUT_LIDAR2_C2 41
#define OUT_LIDAR2_D1 42
#define OUT_LIDAR2_D2 43
#define OUT_LIDAR2_E1 44
#define OUT_LIDAR2_E2 45
#define OUT_LIDAR2_RESTART 46
#define OUT_LIDAR2_OUT08 47
//EZIOCH05 define I8O8P-T
#define OUT_PIO_IN01_VALID 48
#define OUT_PIO_IN02_CS0 49
#define OUT_PIO_IN03_CS1 50
#define OUT_PIO_IN04_CS2 51
#define OUT_PIO_IN05_TR_REQ 52
#define OUT_PIO_IN06_BUSY 53
#define OUT_PIO_IN07_COMPT 54
#define OUT_PIO_IN08_CONT 55
//EZIOCH06 define I8O8P-T
#define CHARGER_MC_ON 56
#define OUT_SPARE_PIN 57
#define OUT_LOWER_FRONT_LEFT_G 58
#define OUT_LOWER_FRONT_LEFT_R 59
#define OUT_LOWER_FRONT_LEFT_B 60
#define OUT_LOWER_FRONT_RIGHT_G 61
#define OUT_LOWER_FRONT_RIGHT_R 62
#define OUT_LOWER_FRONT_RIGHT_B 63

//****ezioID****//
#define ezioID01 0
#define ezioID02 1
#define ezioID03 2
#define ezioID04 3
#define ezioID05 4
#define ezioID06 5

#endif /*_YH_*/




