#ifndef DEFINE_H
#define DEFINE_H

#include <Arduino.h>

#define LED_R 17
#define LED_G 16
#define LED_B 25


#define MODE_INIT               0
#define MODE_LOCAL              1
#define MODE_REMOTE_LOCAL       2
#define MODE_REMOTE             3
#define MODE_DID                4
#define MODE_STOP               5
#define MODE_ERROR              9


//========AUTOWARE========
#define NO_COMMAND                  0
#define AUTONOMOUS                  1           // 全自動駕駛
#define AUTONOMOUS_STEER_ONLY       2           // 測試轉向控制
#define AUTONOMOUS_VELOCITY_ONLY    3           // 測試速度控制
#define MANUAL                      4           // 人工駕駛
#define DISENGAGED                  5           // 駕駛介入後的過渡狀態
#define NOT_READY                   6           // 系統未就緒

#define NONE                        0
#define NEUTRAL                     1
#define DRIVE                       2
#define DRIVE_2                     3
#define DRIVE_3                     4
#define DRIVE_4                     5
#define DRIVE_5                     6
#define DRIVE_6                     7
#define DRIVE_7                     8
#define DRIVE_8                     9
#define DRIVE_9                     10
#define DRIVE_10                    11
#define DRIVE_11                    12
#define DRIVE_12                    13
#define DRIVE_13                    14
#define DRIVE_14                    15
#define DRIVE_15                    16
#define DRIVE_16                    17
#define DRIVE_17                    18
#define DRIVE_18                    19
#define REVERSE                     20
#define REVERSE_2                   21
#define PARK                        22
#define LOW                         23
#define LOW_2                       24

#endif