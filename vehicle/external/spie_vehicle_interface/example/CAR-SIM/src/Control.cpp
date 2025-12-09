#include "Control.h"
#include <Arduino.h>
#include "MCP_MSG.h"

EPAS::EPAS(MCP_MSG& mcp_msg): mcp_msg_(mcp_msg){
  last_get_status_time = millis()-EPAS_STATUS_TIMEOUT;
  last_get_value_time = millis()-EPAS_VALUE_TIMEOUT;
}

// 取得目前EPAS的錯誤狀態
uint8_t EPAS::get_status(){
  return status;
}

// 設定是否啟用ASDS控制
bool EPAS::set_mode(bool enable){
  return ASDS_enable=enable;
}

// 取得目前EPAS的ASDS控制是否啟用
bool EPAS::get_mode(){
  return ADAS_status.ADAS_Enable;
}

// 設定目標角度
int16_t EPAS::set_angle(int16_t angle){
  if(angle>540) angle=540;
  else if(angle<-540) angle=-540;
  target_angle=angle;
  return angle;
}

// 取得目前真實的轉向角度
float EPAS::get_angle(){
  return -current_angle;
}

// 更新，需要在loop()內頻繁呼叫
uint8_t EPAS::update(){
  if(millis()-last_send_time>=EPAS_SEND_LOOP){
    last_send_time=millis();
    if(mcp_msg_.send({ASDS_CTRL_ID, {(uint8_t)(0x01&ASDS_enable), (uint8_t)((target_angle>>8)&0xFF), (uint8_t)(target_angle&0x00FF), 0, 0, 0, 0, 0}, 8})!=CAN_OK)
      return status=EPAS_DISCONNECT;
  }

  if((millis()-last_get_status_time>EPAS_STATUS_TIMEOUT)
      ||(millis()-last_get_value_time>EPAS_VALUE_TIMEOUT)) return status=EPAS_DISCONNECT;
  if(!ADAS_status.ADAS_initialized) return status=EPAS_NOT_INIT;
  if(ADAS_status.ADAS_DID_active){
    set_mode(false);
    return status=EPAS_DID_ACTIVE;
  }

  return status=EPAS_OK;
}

// 當接收到MCP_MSG的資料後，可以傳入這個函式，利用回傳的狀態確認是否為EPAS的訊息
bool EPAS::check(MCP_MSG::Data data){
  if(data.len==8){
    switch(data.id){
      case EPAS_STATUS_ID:
        ADAS_status.ADAS_initialized=data.data[1]&0x80;
        ADAS_status.ADAS_DID_active=data.data[1]&0x40;
        ADAS_status.ADAS_DID_RSTpending=data.data[1]&0x20;
        ADAS_status.ADAS_TimeoutMsg=data.data[2]&0x80;
        ADAS_status.ADAS_RemoteEnable=data.data[2]&0x02;
        ADAS_status.ADAS_Enable=data.data[2]&0x01;
        
        DID_counter=data.data[3];
        RX_msg_Timeout_counter=data.data[4];

        ADAS_setting.ADAS_motorSensorDirection=data.data[6]&0x80;
        ADAS_setting.DID_enabled=data.data[6]&0x40;
        ADAS_setting.ADAS_Func_enable=data.data[7]&0x01;
        last_get_status_time=millis();
        return 1;
        break;
      case EPAS_VALUE_ID:
        current_angle = ((float)((int16_t)((data.data[0]<<8)|data.data[1]))*7)/80;
        current_torque = (int16_t)((data.data[4]<<8)|data.data[5]);
        last_get_value_time=millis();
        return 1;
        break;
    }
  }
  return 0;
}

//======================================================================================

// https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md
VESC::VESC(MCP_MSG& mcp_msg, uint8_t vesc_id): mcp_msg_(mcp_msg), VESC_ID(vesc_id){
  last_get_time=millis()-VESC_TIMEOUT;
}

// 依照指令及VESC ID決定CAN BUS ID
uint32_t VESC::get_send_id(uint8_t command_id) const{
  return (uint32_t)((command_id<<8)|VESC_ID)|0x80000000;
}

// 將int32_t依照指令傳輸
uint8_t VESC::send_data(uint8_t command, int32_t data){
  MCP_MSG::Data data_;
  data_.id=get_send_id(command);
  data_.len=4;
  data_.data[0] = (uint8_t)(data >> 24);
  data_.data[1] = (uint8_t)(data >> 16);
  data_.data[2] = (uint8_t)(data >> 8);
  data_.data[3] = (uint8_t)(data);
  
  return mcp_msg_.send(data_);
}

// 依照指令傳輸指定的資料
uint8_t VESC::send_command(uint8_t command, bool send){
  switch(command){
    case CAN_PACKET_SET_DUTY:
      last_send_command=command;
      if(send&&send_data(command, VESC_DATA.duty)!=CAN_OK)
        return status=VESC_DISCONNECT;
      break;
    case CAN_PACKET_SET_CURRENT:
      last_send_command=command;
      if(send&&send_data(command, VESC_DATA.current)!=CAN_OK)
        return status=VESC_DISCONNECT;
      break;
    case CAN_PACKET_SET_CURRENT_BRAKE:
      last_send_command=command;
      if(send&&send_data(command, VESC_DATA.current_brake)!=CAN_OK)
        return status=VESC_DISCONNECT;
      break;
    case CAN_PACKET_SET_RPM:
      last_send_command=command;
      if(send&&send_data(command, VESC_DATA.rpm)!=CAN_OK)
        return status=VESC_DISCONNECT;
      break;
    case CAN_PACKET_SET_POS:
      last_send_command=command;
      if(send&&send_data(command, VESC_DATA.pos)!=CAN_OK)
        return status=VESC_DISCONNECT;
      break;
    case CAN_PACKET_SET_CURRENT_REL:
      last_send_command=command;
      if(send&&send_data(command, VESC_DATA.current_rel)!=CAN_OK)
        return status=VESC_DISCONNECT;
      break;
    case CAN_PACKET_SET_CURRENT_BRAKE_REL:
      last_send_command=command;
      if(send&&send_data(command, VESC_DATA.current_brake_rel)!=CAN_OK)
        return status=VESC_DISCONNECT;
      break;
    case CAN_PACKET_SET_CURRENT_HANDBRAKE:
      last_send_command=command;
      if(send&&send_data(command, VESC_DATA.current_handbrake)!=CAN_OK)
        return status=VESC_DISCONNECT;
      break;
    case CAN_PACKET_SET_CURRENT_HANDBRAKE_REL:
      last_send_command=command;
      if(send&&send_data(command, VESC_DATA.current_handbrake_rel)!=CAN_OK)
        return status=VESC_DISCONNECT;
      break;
  }
  return VESC_OK;
}

// 回傳目前狀態
uint8_t VESC::get_status(void){
  return status;
}

// 設定各類數值
uint8_t VESC::set_duty(float duty, bool send){
  VESC_DATA.duty=(int32_t)(duty*100000);
  return send_command(CAN_PACKET_SET_DUTY, send);
}
uint8_t VESC::set_current(float current, bool send){
  VESC_DATA.current=(int32_t)(current*1000);
  return send_command(CAN_PACKET_SET_CURRENT, send);
}
uint8_t VESC::set_current_brake(float current_brake, bool send){
  VESC_DATA.current_brake=(int32_t)(current_brake*1000);
  return send_command(CAN_PACKET_SET_CURRENT_BRAKE, send);
}
uint8_t VESC::set_rpm(int32_t rpm, bool send){
  VESC_DATA.rpm=rpm;
  return send_command(CAN_PACKET_SET_RPM, send);
}
uint8_t VESC::set_pos(float pos, bool send){
  VESC_DATA.pos=(int32_t)(pos*1000000);
  return send_command(CAN_PACKET_SET_POS, send);
}
uint8_t VESC::set_current_rel(float current_rel, bool send){
  VESC_DATA.current_rel=(int32_t)(current_rel*100000);
  return send_command(CAN_PACKET_SET_CURRENT_REL, send);
}
uint8_t VESC::set_current_brake_rel(float current_brake_rel, bool send){
  VESC_DATA.current_brake_rel=(int32_t)(current_brake_rel*100000);
  return send_command(CAN_PACKET_SET_CURRENT_BRAKE_REL, send);
}
uint8_t VESC::set_current_handbrake(float current_handbrake, bool send){
  VESC_DATA.current_handbrake=(int32_t)(current_handbrake*1000);
  return send_command(CAN_PACKET_SET_CURRENT_HANDBRAKE, send);
}
uint8_t VESC::set_current_handbrake_rel(float current_handbrake_rel, bool send){
  VESC_DATA.current_handbrake_rel=(int32_t)(current_handbrake_rel*100000);
  return send_command(CAN_PACKET_SET_CURRENT_HANDBRAKE_REL, send);
}

// 更新
uint8_t VESC::update(){
  if(millis()-last_send_time>=VESC_SEND_LOOP){
    last_send_time=millis();
    send_command(last_send_command);
  }

  if(millis()-last_get_time>VESC_TIMEOUT) return status=VESC_DISCONNECT;

  return status=VESC_OK;
}

// 將uint8_t*轉為int16或int32
int16_t VESC::buf2int16(uint8_t* buffer){
  return ((int16_t)buffer[0]<<8 | (int16_t)buffer[1]);
}
int32_t VESC::buf2int32(uint8_t* buffer){
  return ((int32_t)buffer[0]<<24 | (int32_t)buffer[1]<<16 | (int32_t)buffer[2]<<8 | (int32_t)buffer[3]);
}

// 檢查讀到的資料
bool VESC::check(MCP_MSG::Data data){
  if(data.len==8){
    if(data.id==get_send_id(CAN_PACKET_STATUS)){
      last_get_time=millis();
      VESC_STATUS.ERPM=buf2int32(data.data+0);
      VESC_STATUS.Current=(float)(buf2int16(data.data+4))/10;
      VESC_STATUS.Duty_Cycle=(float)(buf2int16(data.data+6))/1000;
      return 1;
    }else if(data.id==get_send_id(CAN_PACKET_STATUS_2)){
      last_get_time=millis();
      VESC_STATUS.Amp_Hours=(float)(buf2int32(data.data+0))/10000;
      VESC_STATUS.Amp_Hours_Chg=(float)(buf2int32(data.data+4))/10000;
      return 1;
    }else if(data.id==get_send_id(CAN_PACKET_STATUS_3)){
      last_get_time=millis();
      VESC_STATUS.Watt_Hours=(float)(buf2int32(data.data+0))/10000;
      VESC_STATUS.Watt_Hours_Chg=(float)(buf2int32(data.data+4))/10000;
      return 1;
    }else if(data.id==get_send_id(CAN_PACKET_STATUS_4)){
      last_get_time=millis();
      VESC_STATUS.Temp_FET=(float)(buf2int16(data.data+0))/10;
      VESC_STATUS.Temp_Motor=(float)(buf2int16(data.data+2))/10;
      VESC_STATUS.Current_In=(float)(buf2int16(data.data+4))/10;
      VESC_STATUS.PID_Pos=(float)(buf2int16(data.data+6))/50;
      return 1;
    }else if(data.id==get_send_id(CAN_PACKET_STATUS_6)){
      last_get_time=millis();
      VESC_STATUS.ADC1=(float)(buf2int16(data.data+0))/1000;
      VESC_STATUS.ADC2=(float)(buf2int16(data.data+2))/1000;
      VESC_STATUS.ADC3=(float)(buf2int16(data.data+4))/1000;
      VESC_STATUS.PPM=(float)(buf2int16(data.data+6))/1000;
      return 1;
    }
  }else if(data.len==6){
    if(data.id==get_send_id(CAN_PACKET_STATUS_5)){
      last_get_time=millis();
      VESC_STATUS.Tachometer=(float)(buf2int32(data.data+0))/6;
      VESC_STATUS.Volts_In=(float)(buf2int16(data.data+4))/10;
      return 1;
    }
  }
  return 0;
}


//======================================================================================

Button::Button(uint8_t pin){
  pinMode(pin_, INPUT);
  pin_= pin;
  use_pin_=true;
}
Button::Button(){}

void Button::check_status(){
  onup_=(!status_&&last_status_);
  ondown_=(status_&&!last_status_);
  last_status_=status_;
}

void Button::update(){
  if(!use_pin_) return;
  status_=digitalRead(pin_);
  check_status();
}
void Button::update(bool status){
  if(use_pin_) return;
  status_=status;
  check_status();
}


//======================================================================================

Pedal::Pedal(uint8_t acce_pin, uint8_t brake_pin){
  acce_pin_=acce_pin;
  brake_pin_=brake_pin;

  last_read_time_=millis();
}

void Pedal::update(){
  acce_status_=analogRead(acce_pin_);
  brake_status_=digitalRead(brake_pin_);

  dt_=millis()-last_read_time_;
  last_read_time_=millis();
}

//======================================================================================
