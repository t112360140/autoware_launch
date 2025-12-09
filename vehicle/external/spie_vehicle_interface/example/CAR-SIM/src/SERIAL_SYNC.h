#ifndef SERIAL_SYNC_H
#define SERIAL_SYNC_H

#include <Arduino.h>
#include "MCP_MSG.h"


/*
    Send Data Struct:
    | HEADER ||         DATA         ||END|
    01 2345 6 01 23 45 67 89 01 23 45 0 12
    xx xxxx x xx xx xx xx xx xx xx xx x xx
    |  |    | |                       | |--- END    (STR)
    |  |    | |                       |----- HASH   (RAW)
    |  |    | |----------------------------- DATA   (HEX)
    |  |    |------------------------------- LEN    (HEX)
    |  |------------------------------------ ID     (HEX)
    |--------------------------------------- START  (STR)

    HASH = ID + LEN + DATA

    ex.
        STR: 'D\t000180102030405060708\x1DE;'
        BUF: [68, 9, 48, 48, 48, 49, 56, 48, 49, 48, 50, 48, 51, 48, 52, 48, 53, 48, 54, 48, 55, 48, 56, 29, 69, 59] 
        DATA CONTENT -> ID: 1, len: 8, DATA: 1 2 3 4 5 6 7 8

*/

#define SERIAL_OK           0x00
#define SERIAL_DISCONNECT   0x01

class SERIAL_SYNC{
public:
    struct Data {
        uint16_t id;
        uint8_t data[15];
        uint8_t len;
    };

private:
    Stream* serial_;

    uint8_t stat_=0;
    uint8_t last_byte=0;
    uint8_t hash_=0;

    struct Data GET_DATA;
    void (*event_)(Data);

    uint8_t hex2bit(uint8_t hex);
    uint8_t bit2hex(uint8_t bit);

    uint32_t last_get_time;
    uint32_t last_send_time;
    uint32_t timeout=100;
    uint32_t loop_time=50;

public:
    SERIAL_SYNC(Stream& serial, uint32_t loop_time_=50, uint32_t timeout=100);

    void setEvent(void (*callback)(Data)){event_ = callback;}

    void update();
    void send(Data data);
    uint8_t get_status(void);

    void send_mcp(MCP_MSG::Data data);
};

#endif