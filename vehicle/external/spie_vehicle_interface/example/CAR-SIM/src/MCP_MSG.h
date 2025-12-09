#ifndef MCP_MSG_h
#define MCP_MSG_h

#include "mcp_can.h"
#include <SPI.h>

class MCP_MSG{
public:
  struct Data{
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
  };
  
private:
  MCP_CAN CAN_;
  
  int INT_;
  
  void (*_event)(Data);
  struct Data MCP_DATA;
  
public:
  MCP_MSG(int CS, int INT);

  uint8_t begin(uint8_t speed, uint8_t clock, uint8_t opMode);
  void setEvent(void (*callback)(Data));

  uint8_t send(Data send_data);
  void check(void);

  uint8_t init_Mask(uint8_t num, uint32_t data){return CAN_.init_Mask(num, data);}
  uint8_t init_Filt(uint8_t num, uint32_t data){return CAN_.init_Filt(num, data);}
};


#define MCP_STATUS_OK            0x00
#define MCP_STATUS_DISCONNECT    0x01
class MCP_GET{
private:
  MCP_MSG& mcp_msg_;
  uint32_t id_;

  uint8_t status_=MCP_STATUS_DISCONNECT;

  void (*_event)(MCP_MSG::Data);

  uint32_t last_get_time_=0;
  uint32_t timeout_=50;       //ms

public:
  MCP_GET(MCP_MSG& mcp_msg, uint32_t id);

  void set_timeout(uint32_t timeout){timeout_=timeout;}
  void setEvent(void (*callback)(MCP_MSG::Data)){_event = callback;}

  uint8_t get_status(void){return status_;}
  uint8_t update(void);

  bool check(MCP_MSG::Data data);
};

class MCP_SEND{
private:
  MCP_MSG& mcp_msg_;

  uint32_t id_;

  uint8_t status_=MCP_STATUS_DISCONNECT;
  bool data_inited_=false;

  uint32_t last_send_time_=0;
  uint32_t loop_time_=50;       //ms

  MCP_MSG::Data last_send_;

public:
  MCP_SEND(MCP_MSG& mcp_msg, uint32_t id);

  uint32_t set_looptime(uint32_t time){return loop_time_=time;}

  uint8_t get_status(void){return status_;}
  uint8_t update(void);

  uint8_t send(MCP_MSG::Data data);
};

#endif
