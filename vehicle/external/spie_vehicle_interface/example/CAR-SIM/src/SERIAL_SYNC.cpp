#include "SERIAL_SYNC.h"
#include <Arduino.h>

SERIAL_SYNC::SERIAL_SYNC(Stream& serial, uint32_t loop_time_, uint32_t timeout_): event_(nullptr){
    serial_=&serial;
    loop_time=loop_time_;
    timeout=timeout_;
    last_get_time = millis()-timeout_;
}

uint8_t SERIAL_SYNC::hex2bit(uint8_t hex){
    if('0'<=hex&&hex<='9') return hex-'0';
    if('a'<=hex&&hex<='f') return hex-'a'+10;

    return 0xFF;
}

uint8_t SERIAL_SYNC::bit2hex(uint8_t bit){
    if(0<=bit&&bit<=9) return bit+'0';
    if(10<=bit&&bit<=15) return bit-10+'a';

    return 0xFF;
}

void SERIAL_SYNC::update(){
    uint8_t jump=0xFF;
    while((jump--)&&serial_->available()){
        uint8_t get_byte=serial_->read();

        if(last_byte=='D'&&get_byte=='\t'){
            hash_=0;
            stat_=1;
        }
        else{
            if(1<=stat_&&stat_<=4){
                if(stat_==1) GET_DATA.id=0;
                GET_DATA.id=GET_DATA.id<<4|hex2bit(get_byte);
                stat_++;
                hash_+=get_byte;
            }else if(stat_==5){
                GET_DATA.len=hex2bit(get_byte);
                if(GET_DATA.len>8) stat_=0;
                else stat_++;
                hash_+=get_byte;
            }else if(6<=stat_&&stat_<6+GET_DATA.len*2){
                uint8_t i=(stat_-6)/2;
                if(stat_%2==0) GET_DATA.data[i]=0;
                GET_DATA.data[i]=GET_DATA.data[i]<<4|hex2bit(get_byte);
                if(hex2bit(get_byte)==0xFF) stat_=0;
                else stat_++;
                hash_+=get_byte;
            }else if(stat_==6+GET_DATA.len*2){
                if(hash_!=get_byte) stat_=0;
                else stat_=98;
            }else if(last_byte=='E'&&get_byte==';'){
                if(stat_==98&&event_!=nullptr) event_(GET_DATA);
                last_get_time=millis();
                stat_=0;
            }
        }

        last_byte=get_byte;
    }
    if(millis()-last_send_time>loop_time){
        send({0x000, {0, 0, 0, 0, 0, 0, 0, 0}, 0});
    }
}

uint8_t SERIAL_SYNC::get_status(void){
    if(!serial_) return SERIAL_DISCONNECT;
    if(millis()-last_get_time>timeout) return SERIAL_DISCONNECT;
    return SERIAL_OK;
}

void SERIAL_SYNC::send(Data data){
    uint8_t send_buf[64]={0};
    int ind=0;
    uint8_t hash=0;

    send_buf[ind]='D';ind++;
    send_buf[ind]='\t';ind++;

    uint16_t id=data.id;
    for(int i=3;i>=0;i--){
        hash+=send_buf[ind+i]=bit2hex(id&0x0F);
        id>>=4;
    }
    ind+=4;
    uint8_t len=data.len<=15?data.len:15;
    hash+=send_buf[ind]=bit2hex(len);ind++;
    for(int i=0;i<len;i++){
        hash+=send_buf[ind]=bit2hex(data.data[i]>>4);ind++;
        hash+=send_buf[ind]=bit2hex(data.data[i]&0x0F);ind++;
    }
    send_buf[ind]=hash;ind++;

    send_buf[ind]='E';ind++;
    send_buf[ind]=';';ind++;
    send_buf[ind]='\n';ind++;

    if(serial_->availableForWrite() > ind+10){
        serial_->write(send_buf, ind);
        last_send_time = millis();
    }
}

void SERIAL_SYNC::send_mcp(MCP_MSG::Data data){
  Data serial_data;
  serial_data.id=data.id&0xFFFF;
  serial_data.len=data.len;
  memcpy(serial_data.data, data.data, serial_data.len);
  send(serial_data);
}