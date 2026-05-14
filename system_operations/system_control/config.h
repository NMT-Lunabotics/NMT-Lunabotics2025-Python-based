#pragma once
#ifndef CONFIG_H
#define CONFIG_H

#define MAIN_ROBOT 1 // REGULUS settings=0, BERMINATOR settings=1, NUC settings=2 
#define PCB_BOARD_SCHEMATIC 0 // REGULUS=0, BERMINATOR=1

//------------------------------------------------------------
//        MAIN ROBOT SETTINGS (REGULUS and BERMINATOR)
//------------------------------------------------------------
#if MAIN_ROBOT==0 || MAIN_ROBOT==1

  // List of components flags to enable/disable for testing
  #define ERROR_LEDS_ENABLED           1
  #define MOTORS_ENABLED               1
  #define BUCKET_ACTUATOR_ENABLED      1
  #define ARM_ACTUATORS_ENABLED        1
  #define SERVO_MOTOR_ENABLED          1
  #define IMU_SENSOR_ENABLED           0
  #define IBUS_RECIVER_ENABLED         0
  #define SCREEN_ENABLED               0

  // List of faults to disable
  #define SERIAL_COMM_TIMEOUT_FAULT    0 //REMOVE, SHUTS DOWN WHOLE SYSTEM IF SERIAL IS NOT RECIVED (COMPONENT_TIMEOUT_FAULTS) IS BETTER IT JUST PUTS SYSTEM INTO IDLE MODE
  #define COMPONENT_TIMEOUT_FAULTS     1 //1

  // Debug mode flags
  #define DEBUG_MODE                   0
  #define SENSOR_OUTPUT                0 // 1: IMU, 2: IBUS, 3: IBUS raw, 4: Final IBUS commands, 5: Actuator positions, 6: Actuator pot values, 8: Servo angle

#else
//------------------------------------------------------------
//                    NUC ROBOT SETTINGS
//------------------------------------------------------------
  #define ERROR_LEDS_ENABLED           0
  #define MOTORS_ENABLED               1
  #define BUCKET_ACTUATOR_ENABLED      0
  #define ARM_ACTUATORS_ENABLED        0
  #define SERVO_MOTOR_ENABLED          1
  #define IMU_SENSOR_ENABLED           0
  #define IBUS_RECIVER_ENABLED         0
  #define SCREEN_ENABLED               0
  #define SERIAL_COMM_TIMEOUT_FAULT    1
  #define COMPONENT_TIMEOUT_FAULTS     0
  #define DEBUG_MODE                   0
  #define SENSOR_OUTPUT                0  
#endif

#endif