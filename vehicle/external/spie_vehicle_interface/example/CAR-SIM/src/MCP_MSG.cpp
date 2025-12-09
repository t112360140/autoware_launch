#include "MCP_MSG.h"
#include <Arduino.h>
#include "mcp_can.h"


MCP_MSG::MCP_MSG(int CS, int INT): CAN_(CS), INT_(INT), _event(nullptr){
  pinMode(INT_, INPUT);
}

// 初始化MCP_CAN
uint8_t MCP_MSG::begin(uint8_t speed, uint8_t clock, uint8_t opMode){
  if(CAN_.begin(MCP_ANY, speed, clock) == CAN_OK) {
    CAN_.setMode(opMode);
    return CAN_OK;
  }
  return CAN_FAILINIT;
}

// 設定當收到訊息時要觸發的函式
void MCP_MSG::setEvent(void (*callback)(Data)){
  _event = callback;
}

// 傳送MCP_CAN資料
uint8_t MCP_MSG::send(Data send_data){
  return CAN_.sendMsgBuf(send_data.id, send_data.len, send_data.data);
}

// 檢查MCP的狀態，如果收到訊息，則觸發函式，需要在loop()內頻繁呼叫
void MCP_MSG::check(void){
  while(!digitalRead(INT_)){
    CAN_.readMsgBuf(&MCP_DATA.id, &MCP_DATA.len, MCP_DATA.data);
    if(_event!=nullptr) _event(MCP_DATA);
  }
}


MCP_GET::MCP_GET(MCP_MSG& mcp_msg, uint32_t id): mcp_msg_(mcp_msg), id_(id), _event(nullptr){
  last_get_time_=millis();
}

uint8_t MCP_GET::update(){
  if(millis()-last_get_time_>timeout_) return status_=MCP_STATUS_DISCONNECT;
  return status_;
}

bool MCP_GET::check(MCP_MSG::Data data){
  if(data.id==id_){
    last_get_time_=millis();
    status_=MCP_STATUS_OK;
    if(_event!=nullptr) _event(data);
    return 1;
  }
  return 0;
}

MCP_SEND::MCP_SEND(MCP_MSG& mcp_msg, uint32_t id): mcp_msg_(mcp_msg), id_(id){
  
}

uint8_t MCP_SEND::update(){
  if(data_inited_&&millis()-last_send_time_>=loop_time_){
    last_send_time_=millis();
    if(mcp_msg_.send(last_send_)!=CAN_OK)
      return status_=MCP_STATUS_DISCONNECT;
    return status_=MCP_STATUS_OK;
  }
  return status_;
}

uint8_t MCP_SEND::send(MCP_MSG::Data data){
  last_send_time_=millis();
  last_send_.id=id_;
  memcpy(last_send_.data, data.data, data.len);
  last_send_.len=data.len;
  data_inited_=true;

  if(mcp_msg_.send(last_send_)!=CAN_OK)
    return status_=MCP_STATUS_DISCONNECT;
  return status_=MCP_STATUS_OK;
}