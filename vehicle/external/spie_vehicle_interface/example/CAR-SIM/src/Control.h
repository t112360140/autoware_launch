#ifndef CONTROL_h
#define CONTROL_h

#include "MCP_MSG.h"

#define ASDS_CTRL_ID    0x415
#define EPAS_STATUS_ID  0x141
#define EPAS_VALUE_ID   0xC1

#define EPAS_SEND_LOOP        20    //ms
#define EPAS_STATUS_TIMEOUT   120   //ms
#define EPAS_VALUE_TIMEOUT    50    //ms

#define EPAS_OK         0x00
#define EPAS_DID_ACTIVE 0x01
#define EPAS_DISCONNECT 0x02
#define EPAS_NOT_INIT   0x03

class EPAS{
private:
  MCP_MSG& mcp_msg_;

  uint32_t last_get_status_time=0;
  uint32_t last_get_value_time=0;

  uint8_t status=EPAS_DISCONNECT;

  bool ASDS_enable=false;
  int16_t target_angle=0;

  float current_angle=0;
  int16_t current_torque=0;

  uint8_t DID_counter=0;
  uint8_t RX_msg_Timeout_counter=0;

  struct {
    bool ADAS_initialized=false;
    bool ADAS_DID_active=false;
    bool ADAS_DID_RSTpending=false;
    bool ADAS_TimeoutMsg=false;
    bool ADAS_RemoteEnable=false;
    bool ADAS_Enable=false;
  } ADAS_status;
  struct {
    bool ADAS_motorSensorDirection=false;
    bool DID_enabled=false;
    bool ADAS_Func_enable=false;
  } ADAS_setting;
  
  uint32_t last_send_time=0;
  
public:
  EPAS(MCP_MSG& mcp_msg);

  uint8_t get_status(void);
  bool check(MCP_MSG::Data data);
  uint8_t update();

  bool set_mode(bool enable);
  bool get_mode();

  int16_t set_angle(int16_t angle);
  float get_angle();
};


//======================================================================================
#define VESC_SEND_LOOP        50    //ms
#define VESC_TIMEOUT          70   //ms

#define VESC_OK         0x00
#define VESC_DISCONNECT 0x01
#define VESC_ERROR      0x02

#define CAN_PACKET_SET_DUTY                   0x00
#define CAN_PACKET_SET_CURRENT                0x01
#define CAN_PACKET_SET_CURRENT_BRAKE          0x02
#define CAN_PACKET_SET_RPM                    0x03
#define CAN_PACKET_SET_POS                    0x04
#define CAN_PACKET_SET_CURRENT_REL            0x0A
#define CAN_PACKET_SET_CURRENT_BRAKE_REL      0x0B
#define CAN_PACKET_SET_CURRENT_HANDBRAKE      0x0C
#define CAN_PACKET_SET_CURRENT_HANDBRAKE_REL  0x0D

#define CAN_PACKET_STATUS                     0x09
#define CAN_PACKET_STATUS_2                   0x0E
#define CAN_PACKET_STATUS_3                   0x0F
#define CAN_PACKET_STATUS_4                   0x10
#define CAN_PACKET_STATUS_5                   0x1B
#define CAN_PACKET_STATUS_6                   0x3A

#define CAN_PACKET_NULL                       0xFF

#define VESC_MOTOR_MAX  0
#define MAX_RPM         0

class VESC{
private:
  MCP_MSG& mcp_msg_;

  const uint8_t VESC_ID;

  uint8_t status=VESC_DISCONNECT;

  uint32_t get_send_id(uint8_t command_id) const;
  uint8_t send_data(uint8_t command, int32_t data);
  uint8_t send_command(uint8_t command, bool send=true);

  uint8_t last_send_command=CAN_PACKET_NULL;

  uint32_t last_get_time=0;
  uint32_t last_send_time=0;

  struct {
    int32_t duty=0;
    int32_t current=0;
    int32_t current_brake=0;
    int32_t rpm=0;
    int32_t pos=0;
    int32_t current_rel=0;
    int32_t current_brake_rel=0;
    int32_t current_handbrake=0;
    int32_t current_handbrake_rel=0;
  } VESC_DATA;

  int16_t buf2int16(uint8_t* buffer);
  int32_t buf2int32(uint8_t* buffer);

  struct {
    int32_t ERPM=0;         // CAN_PACKET_STATUS
    float Current=0;
    float Duty_Cycle=0;
    float Amp_Hours=0;      // CAN_PACKET_STATUS_2
    float Amp_Hours_Chg=0;
    float Watt_Hours=0;     // CAN_PACKET_STATUS_3
    float Watt_Hours_Chg=0;
    float Temp_FET=0;       // CAN_PACKET_STATUS_4
    float Temp_Motor=0;
    float Current_In=0;
    float PID_Pos=0;
    float ADC1=0;           // CAN_PACKET_STATUS_6
    float ADC2=0;
    float ADC3=0;
    float PPM=0;
    float Tachometer=0;     // CAN_PACKET_STATUS_5
    float Volts_In=0;
  } VESC_STATUS;
  
public:
  VESC(MCP_MSG& mcp_msg, uint8_t vesc_id);

  uint8_t get_status(void);
  bool check(MCP_MSG::Data data);
  uint8_t update();

  uint8_t get_vesc_id(void){return VESC_ID;}

  uint8_t set_duty(float duty, bool send=false);
  uint8_t set_current(float current, bool send=false);
  uint8_t set_current_brake(float current_brake, bool send=false);
  uint8_t set_rpm(int32_t rpm, bool send=false);
  uint8_t set_pos(float pos, bool send=false);
  uint8_t set_current_rel(float current_rel, bool send=false);
  uint8_t set_current_brake_rel(float current_brake_rel, bool send=false);
  uint8_t set_current_handbrake(float current_handbrake, bool send=false);
  uint8_t set_current_handbrake_rel(float current_handbrake_rel, bool send=false);

  float get_duty(void){return VESC_STATUS.Duty_Cycle;}
  float get_current(void){return VESC_STATUS.Current;}
};


//======================================================================================

class Button{
private:
  bool use_pin_=false;
  uint8_t pin_=0;
  bool status_=0;
  bool last_status_=0;
  bool onup_=false;
  bool ondown_=false;

  void check_status(void);

public:
  Button(void);
  Button(uint8_t pin);

  void update(void);
  void update(bool status);

  void clear(void){onup_=0;ondown_=0;};

  bool OnClick(void){return status_;}
  bool OnUp(void){return onup_;}
  bool OnDown(void){return ondown_;}
};


//======================================================================================

class Pedal{
private:
  uint8_t acce_pin_=0;
  uint8_t brake_pin_=0;

  uint16_t acce_status_=0;
  bool brake_status_=0;

  uint32_t last_read_time_=0;
  uint16_t dt_=0;

public:
  Pedal(uint8_t acce_pin, uint8_t brake_pin);

  void update(void);

  uint16_t get_acce(void){return acce_status_;}
  bool get_brake(void){return brake_status_;}
};


//======================================================================================


#endif
