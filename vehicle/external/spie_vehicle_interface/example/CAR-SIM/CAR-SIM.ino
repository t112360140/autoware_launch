#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>  // https://github.com/adafruit/Adafruit_SH110x

#include "src/Timer.h"
#include "src/SERIAL_SYNC.h"
#include "Define.h"


const int MUXPin[] = {D2, D3, D6, D7};
bool analog_pin[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0};
int16_t button_status[16] = {0};


Timer LED_CTL;
void led_ctl();

Timer MSG_LOOP;
void msg_loop();

SERIAL_SYNC serial_sync(Serial);
void get_serial(SERIAL_SYNC::Data data);

struct {
  float Speed=0;
  float Angle=0;
  uint8_t Gear=0;
  uint8_t TurnIndicators=0;
  uint8_t HazardLights=0;
  bool Engage=false;
  bool Emergency=false;
} REMOTE_CTRL;

struct {
  float Speed=0;
  float Angle=0;
  uint8_t Gear=0;
  uint8_t TurnIndicators=0;
  uint8_t HazardLights=0;
} LOCAL_CTRL;

struct {
  float Speed=0;
  float Angle=0;
  uint8_t Gear=0;
  uint8_t TurnIndicators=0;
  uint8_t HazardLights=0;
} LOCAL_STATUS;

void setup() {
  pinMode(LED_R, OUTPUT);
  digitalWrite(LED_R, 1);
  pinMode(LED_G, OUTPUT);
  digitalWrite(LED_G, 1);
  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_B, 1);

  pinMode(D1, INPUT);
  for (int i = 0; i < 4; i++) {
      pinMode(MUXPin[i], OUTPUT);
  }
  
  Serial.begin(115200);
//  while(!Serial);
  Serial.print("----------------------------------------\n");
  Serial.print("[INFO]\tInit Start.\n\n");

  Serial.print("[INFO]\tSet Serial Sync.\n");
  serial_sync.setEvent(get_serial);

  Serial.print("[INFO]\tSet LED Loop.\n");
  LED_CTL.set(200, led_ctl);

  Serial.print("[INFO]\tSet MSG Loop(test).\n");
  MSG_LOOP.set(20, msg_loop);

  Serial.print("\n[INFO]\tInit End.\n");
  Serial.print("----------------------------------------\n");
}

void loop() {

  serial_sync.update();      // 更新序列傳輸(接收資料&keep-alive)

  CONTROL();    // 主要流程

  MSG_LOOP.update();      // 傳輸資料
  LED_CTL.update();       // LED控制
}

uint8_t MODE_STATUS=NOT_READY;
void CONTROL(){
  switch(MODE_STATUS){
    case NOT_READY:

      break;
    case MANUAL:
      LOCAL_STATUS.Speed=LOCAL_CTRL.Speed;
      LOCAL_STATUS.Angle=LOCAL_CTRL.Angle;
      break;
    case AUTONOMOUS:
      LOCAL_STATUS.Speed=REMOTE_CTRL.Speed;
      LOCAL_STATUS.Angle=REMOTE_CTRL.Angle;
      break;
    default:
      MODE_STATUS=DISENGAGED;
      break;
  }
}

void get_serial(SERIAL_SYNC::Data data){
  switch(data.id){
    case 0x100:
      if(data.len==8){
        REMOTE_CTRL.Angle = ((float*)(data.data+0))[0];
        REMOTE_CTRL.Speed = ((float*)(data.data+4))[0];
      }
      break;
    case 0x101:
      if(data.len==1){
        REMOTE_CTRL.Gear = data.data[0];
      }
      break;
    case 0x102:
      if(data.len==1){
        REMOTE_CTRL.TurnIndicators = data.data[0];
      }
      break;
    case 0x103:
      if(data.len==1){
        REMOTE_CTRL.HazardLights = data.data[0];
      }
      break;
    case 0x104:
      if(data.len==1){
        REMOTE_CTRL.Engage = data.data[0];
      }
      break;
    case 0x105:
      if(data.len==1){
        REMOTE_CTRL.Emergency = data.data[0];
      }
      break;
  }
}

void msg_loop(){
  SERIAL_SYNC::Data data;

  data.id=0x110;
  data.len=1;
  data.data[0]=MODE_STATUS;
  serial_sync.send(data);

  data.id=0x111;
  data.len=4;
  uint8_t* value=(uint8_t*)(&LOCAL_STATUS.Angle);
  memory(data.data+0, value, 4);
  serial_sync.send(data);

  data.id=0x112;
  data.len=4;
  uint8_t* value=(uint8_t*)(&LOCAL_STATUS.Angle);
  memory(data.data+0, value, 4);
  serial_sync.send(data);

  data.id=0x113;
  data.len=1;
  data.data[0]=LOCAL_STATUS.Gear;
  serial_sync.send(data);

  data.id=0x114;
  data.len=1;
  data.data[0]=LOCAL_STATUS.TurnIndicators;
  serial_sync.send(data);

  data.id=0x115;
  data.len=1;
  data.data[0]=LOCAL_STATUS.HazardLights;
  serial_sync.send(data);
}

void led_ctl(){
  switch(MODE_STATUS){
    case MODE_INIT:
      digitalWrite(LED_R, 0);
      digitalWrite(LED_G, 0);
      digitalWrite(LED_B, 0);
      break;
    case MODE_LOCAL:
      digitalWrite(LED_R, 1);
      digitalWrite(LED_G, 0);
      digitalWrite(LED_B, 1);
      break;
    case MODE_REMOTE_LOCAL:
      digitalWrite(LED_R, 1);
      digitalWrite(LED_G, !digitalRead(LED_G));
      digitalWrite(LED_B, !digitalRead(LED_B));
      break;
    case MODE_REMOTE:
      digitalWrite(LED_R, 1);
      digitalWrite(LED_G, 1);
      digitalWrite(LED_B, 0);
      break;
    case MODE_ERROR:
      digitalWrite(LED_R, !digitalRead(LED_R));
      digitalWrite(LED_G, 1);
      digitalWrite(LED_B, 1);
      break;
  }
}

void read_button() {
  for (int i = 0; i < 15; i++) {
    char mask = 0b0001;
    for (int j = 0; j < 4; j++) {
      digitalWrite(MUXPin[j], i & mask);
      mask = mask << 1;
    }
    analogRead(D1);
    int value = analogRead(D1);
    if (analog_pin[i]) button_status[i] = value;
    else button_status[i] = !(value > 512);
  }
}

void setup1(){
  delay(250); // wait for the OLED to power up
  display_L.begin(i2c_Address_L, true);
  display_R.begin(i2c_Address_R, true);
  display_L.display();
  display_R.display();
  delay(500);
  display_L.clearDisplay();
  display_R.clearDisplay();
  display_L.display();
  display_R.display();
  
  display_L.setTextSize(1);
  display_L.setTextColor(SH110X_WHITE);
  display_R.setTextSize(1);
  display_R.setTextColor(SH110X_WHITE);
}

void loop1(){
  read_button();

  display_L.clearDisplay();
  display_L.setCursor(0,0);
  display_L.print("MODE: ");
  display_L.println(((mode==0)?"INIT":(mode==1)?"LOCAL":(mode==2)?"REMOTE_LOCAL":(mode==3)?"REMOTE":"ERROR"));
  display_L.print("TA:");
  display_L.print(angle);
  if(angle>=0) display_L.print(" ");if(angle/10==0) display_L.print(" ");
  if(angle/100==0) display_L.print(" ");if(angle/1000==0) display_L.print(" ");
  display_L.print(" CA:");
  display_L.println(c_angle);
  display_L.print("STATUS: ");
  display_L.println(((epas_status==0)?"OK":(epas_status==1)?"DID ACTIVE":(epas_status==2)?"DISCONNECT":"NOT INIT"));
  display_L.print("EPAS MODE: ");
  display_L.println(epas_mode?"ASDS":"LOCAL");
  display_L.print("TD:");
  display_L.print(duty);
  if(duty>=0) display_L.print(" ");
  display_L.print(" CD:");
  display_L.println(c_duty);
  display_L.print("STATUS: ");
  display_L.println(((vesc_status==0)?"OK":(vesc_status==1)?"DISCONNECT":"ERROR"));
  display_L.display();
  
  display_R.clearDisplay();
  display_R.setCursor(0,0);
  for(int i=0;i<4;i++){
    display_R.print(button_status[i]);
    display_R.print(' ');
  }
  display_R.print('\n');
  for(int i=7;i>=4;i--){
    display_R.print(button_status[i]);
    display_R.print(' ');
  }
  display_R.print("\nR: ");
  for(int i=0;i<3;i++){
    display_R.print(button_status[8+i]);
    display_R.print(" ");
  }
  display_R.print("\nL: ");
  for(int i=0;i<3;i++){
    display_R.print(button_status[11+i]);
    display_R.print(" ");
  }
  display_R.print("\n\nCAN id: ");
  display_R.print(rxId, HEX);
  display_R.print(" data:\n");
  for(int i=0;i<8;i++){
    display_R.print(rxBuf[i], HEX);
    display_R.print(" ");
  }
  display_R.display();
}#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>  // https://github.com/adafruit/Adafruit_SH110x

#include "src/Timer.h"
#include "src/SERIAL_SYNC.h"
#include "Define.h"


const int MUXPin[] = {D2, D3, D6, D7};
bool analog_pin[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0};
int16_t button_status[16] = {0};


Timer LED_CTL;
void led_ctl();

Timer MSG_LOOP;
void msg_loop();

SERIAL_SYNC serial_sync(Serial);
void get_serial(SERIAL_SYNC::Data data);

struct {
  float Speed=0;
  float Angle=0;
  uint8_t Gear=0;
  uint8_t TurnIndicators=0;
  uint8_t HazardLights=0;
  bool Engage=false;
  bool Emergency=false;
} REMOTE_CTRL;

struct {
  float Speed=0;
  float Angle=0;
  uint8_t Gear=0;
  uint8_t TurnIndicators=0;
  uint8_t HazardLights=0;
} LOCAL_CTRL;

struct {
  float Speed=0;
  float Angle=0;
  uint8_t Gear=0;
  uint8_t TurnIndicators=0;
  uint8_t HazardLights=0;
} LOCAL_STATUS;

void setup() {
  pinMode(LED_R, OUTPUT);
  digitalWrite(LED_R, 1);
  pinMode(LED_G, OUTPUT);
  digitalWrite(LED_G, 1);
  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_B, 1);

  pinMode(D1, INPUT);
  for (int i = 0; i < 4; i++) {
      pinMode(MUXPin[i], OUTPUT);
  }
  
  Serial.begin(115200);
//  while(!Serial);
  Serial.print("----------------------------------------\n");
  Serial.print("[INFO]\tInit Start.\n\n");

  Serial.print("[INFO]\tSet Serial Sync.\n");
  serial_sync.setEvent(get_serial);

  Serial.print("[INFO]\tSet LED Loop.\n");
  LED_CTL.set(200, led_ctl);

  Serial.print("[INFO]\tSet MSG Loop(test).\n");
  MSG_LOOP.set(20, msg_loop);

  Serial.print("\n[INFO]\tInit End.\n");
  Serial.print("----------------------------------------\n");
}

void loop() {

  serial_sync.update();      // 更新序列傳輸(接收資料&keep-alive)

  CONTROL();    // 主要流程

  MSG_LOOP.update();      // 傳輸資料
  LED_CTL.update();       // LED控制
}

uint8_t MODE_STATUS=NOT_READY;
void CONTROL(){
  switch(MODE_STATUS){
    case NOT_READY:

      break;
    case MANUAL:
      LOCAL_STATUS.Speed=LOCAL_CTRL.Speed;
      LOCAL_STATUS.Angle=LOCAL_CTRL.Angle;
      break;
    case AUTONOMOUS:
      LOCAL_STATUS.Speed=REMOTE_CTRL.Speed;
      LOCAL_STATUS.Angle=REMOTE_CTRL.Angle;
      break;
    default:
      MODE_STATUS=DISENGAGED;
      break;
  }
}

void get_serial(SERIAL_SYNC::Data data){
  switch(data.id){
    case 0x100:
      if(data.len==8){
        REMOTE_CTRL.Angle = ((float*)(data.data+0))[0];
        REMOTE_CTRL.Speed = ((float*)(data.data+4))[0];
      }
      break;
    case 0x101:
      if(data.len==1){
        REMOTE_CTRL.Gear = data.data[0];
      }
      break;
    case 0x102:
      if(data.len==1){
        REMOTE_CTRL.TurnIndicators = data.data[0];
      }
      break;
    case 0x103:
      if(data.len==1){
        REMOTE_CTRL.HazardLights = data.data[0];
      }
      break;
    case 0x104:
      if(data.len==1){
        REMOTE_CTRL.Engage = data.data[0];
      }
      break;
    case 0x105:
      if(data.len==1){
        REMOTE_CTRL.Emergency = data.data[0];
      }
      break;
  }
}

void msg_loop(){
  SERIAL_SYNC::Data data;

  data.id=0x110;
  data.len=1;
  data.data[0]=MODE_STATUS;
  serial_sync.send(data);

  data.id=0x111;
  data.len=4;
  uint8_t* value=(uint8_t*)(&LOCAL_STATUS.Angle);
  memory(data.data+0, value, 4);
  serial_sync.send(data);

  data.id=0x112;
  data.len=4;
  uint8_t* value=(uint8_t*)(&LOCAL_STATUS.Angle);
  memory(data.data+0, value, 4);
  serial_sync.send(data);

  data.id=0x113;
  data.len=1;
  data.data[0]=LOCAL_STATUS.Gear;
  serial_sync.send(data);

  data.id=0x114;
  data.len=1;
  data.data[0]=LOCAL_STATUS.TurnIndicators;
  serial_sync.send(data);

  data.id=0x115;
  data.len=1;
  data.data[0]=LOCAL_STATUS.HazardLights;
  serial_sync.send(data);
}

void led_ctl(){
  switch(MODE_STATUS){
    case MODE_INIT:
      digitalWrite(LED_R, 0);
      digitalWrite(LED_G, 0);
      digitalWrite(LED_B, 0);
      break;
    case MODE_LOCAL:
      digitalWrite(LED_R, 1);
      digitalWrite(LED_G, 0);
      digitalWrite(LED_B, 1);
      break;
    case MODE_REMOTE_LOCAL:
      digitalWrite(LED_R, 1);
      digitalWrite(LED_G, !digitalRead(LED_G));
      digitalWrite(LED_B, !digitalRead(LED_B));
      break;
    case MODE_REMOTE:
      digitalWrite(LED_R, 1);
      digitalWrite(LED_G, 1);
      digitalWrite(LED_B, 0);
      break;
    case MODE_ERROR:
      digitalWrite(LED_R, !digitalRead(LED_R));
      digitalWrite(LED_G, 1);
      digitalWrite(LED_B, 1);
      break;
  }
}

void read_button() {
  for (int i = 0; i < 15; i++) {
    char mask = 0b0001;
    for (int j = 0; j < 4; j++) {
      digitalWrite(MUXPin[j], i & mask);
      mask = mask << 1;
    }
    analogRead(D1);
    int value = analogRead(D1);
    if (analog_pin[i]) button_status[i] = value;
    else button_status[i] = !(value > 512);
  }
}

void setup1(){
  delay(250); // wait for the OLED to power up
  display_L.begin(i2c_Address_L, true);
  display_R.begin(i2c_Address_R, true);
  display_L.display();
  display_R.display();
  delay(500);
  display_L.clearDisplay();
  display_R.clearDisplay();
  display_L.display();
  display_R.display();
  
  display_L.setTextSize(1);
  display_L.setTextColor(SH110X_WHITE);
  display_R.setTextSize(1);
  display_R.setTextColor(SH110X_WHITE);
}

void loop1(){
  read_button();

  display_L.clearDisplay();
  display_L.setCursor(0,0);
  display_L.print("MODE: ");
  display_L.println(((mode==0)?"INIT":(mode==1)?"LOCAL":(mode==2)?"REMOTE_LOCAL":(mode==3)?"REMOTE":"ERROR"));
  display_L.print("TA:");
  display_L.print(angle);
  if(angle>=0) display_L.print(" ");if(angle/10==0) display_L.print(" ");
  if(angle/100==0) display_L.print(" ");if(angle/1000==0) display_L.print(" ");
  display_L.print(" CA:");
  display_L.println(c_angle);
  display_L.print("STATUS: ");
  display_L.println(((epas_status==0)?"OK":(epas_status==1)?"DID ACTIVE":(epas_status==2)?"DISCONNECT":"NOT INIT"));
  display_L.print("EPAS MODE: ");
  display_L.println(epas_mode?"ASDS":"LOCAL");
  display_L.print("TD:");
  display_L.print(duty);
  if(duty>=0) display_L.print(" ");
  display_L.print(" CD:");
  display_L.println(c_duty);
  display_L.print("STATUS: ");
  display_L.println(((vesc_status==0)?"OK":(vesc_status==1)?"DISCONNECT":"ERROR"));
  display_L.display();
  
  display_R.clearDisplay();
  display_R.setCursor(0,0);
  for(int i=0;i<4;i++){
    display_R.print(button_status[i]);
    display_R.print(' ');
  }
  display_R.print('\n');
  for(int i=7;i>=4;i--){
    display_R.print(button_status[i]);
    display_R.print(' ');
  }
  display_R.print("\nR: ");
  for(int i=0;i<3;i++){
    display_R.print(button_status[8+i]);
    display_R.print(" ");
  }
  display_R.print("\nL: ");
  for(int i=0;i<3;i++){
    display_R.print(button_status[11+i]);
    display_R.print(" ");
  }
  display_R.print("\n\nCAN id: ");
  display_R.print(rxId, HEX);
  display_R.print(" data:\n");
  for(int i=0;i<8;i++){
    display_R.print(rxBuf[i], HEX);
    display_R.print(" ");
  }
  display_R.display();
}