#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>  // https://github.com/adafruit/Adafruit_SH110x

#include "src/Timer.h"
#include "src/Control.h"
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

Button Start_Button;
Button PARK_switch;
Button Auto_Switch;
Button EStop;

Button R_light;
Button L_light;

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
  uint8_t Gear=PARK;
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
  Start_Button.update(button_status[0]);
  PARK_switch.update(button_status[1]);
  Auto_Switch.update(button_status[2]);
  EStop.update(button_status[3]);

  R_light.update(button_status[8]);
  L_light.update(button_status[11]);

  // 模擬車輛控制輸入

  LOCAL_CTRL.Angle=(((float)button_status[9]-512)/512)*0.698;
  LOCAL_CTRL.Speed=((512-(float)button_status[13])/512)*6.9;

  if(PARK_switch.onDown()){
    if(LOCAL_CTRL.Gear==PARK) LOCAL_CTRL.Gear=NEUTRAL;
    else if(abs(LOCAL_STATUS.Speed)<0.2) LOCAL_CTRL.Gear=PARK;
  }
  if(LOCAL_CTRL.Gear!=PARK && abs(LOCAL_STATUS.Speed) < 0.2){
    if(LOCAL_CTRL.Speed>0.2) LOCAL_CTRL.Gear=DRIVE;
    else if(LOCAL_CTRL.Speed<-0.2) LOCAL_CTRL.Gear=REVERSE;
  }
  // LOCAL_CTRL.Speed=abs(((512-(float)button_status[10])/512)*6.9)*(LOCAL_CTRL.Gear?-1:1);


  if(LOCAL_CTRL.TurnIndicators==1){
    if(L_light.onDown()) LOCAL_CTRL.TurnIndicators=2;
    else if(R_light.onDown()) LOCAL_CTRL.TurnIndicators=3;
  }else if(LOCAL_CTRL.TurnIndicators==2){
    if(L_light.onDown()) LOCAL_CTRL.TurnIndicators=1;
    else if(R_light.onDown()) LOCAL_CTRL.TurnIndicators=3;
  }else if(LOCAL_CTRL.TurnIndicators==3){
    if(L_light.onDown()) LOCAL_CTRL.TurnIndicators=2;
    else if(R_light.onDown()) LOCAL_CTRL.TurnIndicators=1;
  }else{
    LOCAL_CTRL.TurnIndicators=1;
  }

  //--------

  serial_sync.update();      // 更新序列傳輸(接收資料&keep-alive)

  CONTROL();    // 主要流程

  MSG_LOOP.update();      // 傳輸資料
  LED_CTL.update();       // LED控制

  Start_Button.clear();
  PARK_switch.clear();
  Auto_Switch.clear();
  EStop.clear();

  R_light.clear();
  L_light.clear();
}

uint8_t MODE_STATUS=MODE_INIT;
void CONTROL(){
  if(EStop.onClick()) MODE_STATUS=MODE_ERROR;

  switch(MODE_STATUS){
    case MODE_INIT:

      if(Start_Button.onDown()) MODE_STATUS=MODE_LOCAL;
      break;
    case MODE_LOCAL:
    case MODE_DID:
      LOCAL_STATUS.Gear=LOCAL_CTRL.Gear;
      if(LOCAL_CTRL.Gear==DRIVE) LOCAL_STATUS.Speed=max(LOCAL_CTRL.Speed, 0);
      else if(LOCAL_CTRL.Gear==REVERSE) LOCAL_STATUS.Speed=min(LOCAL_CTRL.Speed, 0);
      else LOCAL_STATUS.Speed=0;
      if(LOCAL_CTRL.Gear!=PARK) LOCAL_STATUS.Angle=LOCAL_CTRL.Angle;

      LOCAL_STATUS.TurnIndicators=LOCAL_CTRL.TurnIndicators;

      if(MODE_STATUS==MODE_LOCAL){
        if(Auto_Switch.onDown()&&serial_sync.get_status()==SERIAL_OK&&abs(LOCAL_STATUS.Speed)<0.2)
          MODE_STATUS=MODE_REMOTE;
      }else{
        if(Auto_Switch.onDown()) MODE_STATUS=MODE_LOCAL;
      }
      break;
    case MODE_REMOTE_LOCAL:
      LOCAL_STATUS.Speed=0;
      LOCAL_STATUS.Gear=PARK;

      if(Auto_Switch.onDown()) MODE_STATUS=MODE_LOCAL;
      if(serial_sync.get_status()==SERIAL_OK) MODE_STATUS=MODE_REMOTE;
      else MODE_STATUS=MODE_ERROR;
      break;
    case MODE_REMOTE:
      if(true||REMOTE_CTRL.Engage){       // 無視了Engage，要找一下原因
        LOCAL_STATUS.Gear=REMOTE_CTRL.Gear;

        if(REMOTE_CTRL.Gear==DRIVE) LOCAL_STATUS.Speed=max(REMOTE_CTRL.Speed, 0);
        else if(REMOTE_CTRL.Gear==REVERSE) LOCAL_STATUS.Speed=min(REMOTE_CTRL.Speed, 0);
        else LOCAL_STATUS.Speed=0;
        LOCAL_STATUS.Angle=REMOTE_CTRL.Angle;
      }else{
        LOCAL_STATUS.Speed=0;

        LOCAL_STATUS.Gear=PARK;
      }

      if(Auto_Switch.onDown()) MODE_STATUS=MODE_LOCAL;
      if(serial_sync.get_status()!=SERIAL_OK) MODE_STATUS=MODE_ERROR;
      if(REMOTE_CTRL.Emergency) MODE_STATUS=MODE_ERROR;
      break;
    case MODE_STOP:

      if(Start_Button.onDown()) MODE_STATUS=MODE_LOCAL;
      break;
    case MODE_ERROR:
      LOCAL_STATUS.Speed=0;
      LOCAL_STATUS.Angle=LOCAL_STATUS.Angle;

      if(Start_Button.onDown()) MODE_STATUS=MODE_LOCAL;
      break;
    default:
      MODE_STATUS=MODE_ERROR;
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
  switch(MODE_STATUS){
    case MODE_LOCAL:
      data.data[0]=MANUAL;
      break;
    case MODE_REMOTE:
      data.data[0]=AUTONOMOUS;
      break;
    case MODE_DID:
      data.data[0]=DISENGAGED;
      break;
    default:
      data.data[0]=NOT_READY;
      break;
  }
  serial_sync.send(data);

  data.id=0x111;
  data.len=8;
  memcpy(data.data+0, &LOCAL_STATUS.Speed, 4);
  memcpy(data.data+4, &LOCAL_STATUS.Angle, 4);
  serial_sync.send(data);

  data.id=0x112;
  data.len=4;
  memcpy(data.data+0, &LOCAL_STATUS.Angle, 4);
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

bool led_timer=false;
void led_ctl(){
  led_timer=!led_timer;

  switch(MODE_STATUS){
    case MODE_INIT:
    case MODE_STOP:
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
    case MODE_DID:
      digitalWrite(LED_R, 1);
      digitalWrite(LED_G, 0);
      digitalWrite(LED_B, 0);
      break;
    default:
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



#define i2c_Address_L 0x3c
#define i2c_Address_R 0x3d
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SH1106G display_L = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SH1106G display_R = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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

void print_float(Adafruit_SH1106G& display, float value){
  if(value>=0) display.print(" ");
  if(abs(value)<10) display.print(" ");
  display.print(value);
}

void loop1(){
  read_button();

  display_L.clearDisplay();
  display_L.setCursor(0,0);
  display_L.print("MODE: ");
  switch(MODE_STATUS){
    case MODE_INIT:
      display_L.print("INIT  ");
      break;
    case MODE_LOCAL:
      display_L.print("LOCAL ");
      break;
    case MODE_REMOTE_LOCAL:
      display_L.print("R->L  ");
      break;
    case MODE_REMOTE:
      display_L.print("REMOTE");
      break;
    case MODE_DID:
      display_L.print("DID   ");
      break;
    default:
      display_L.print("ERROR ");
      break;
  }
  display_L.print(" SER: ");
  display_L.print(serial_sync.get_status()==SERIAL_OK?"OK":"NOT");
  display_L.print("\nSTATUS:     Gear: ");
  if(LOCAL_STATUS.Gear==NEUTRAL) display_L.print("N");
  else if(LOCAL_STATUS.Gear==DRIVE) display_L.print("D");
  else if(LOCAL_STATUS.Gear==REVERSE) display_L.print("R");
  else if(LOCAL_STATUS.Gear==PARK) display_L.print("P");
  else display_L.print("E");
  display_L.print("\nA:");
  print_float(display_L, LOCAL_STATUS.Angle);
  display_L.print(" S:");
  print_float(display_L, LOCAL_STATUS.Speed);
  display_L.print("\nLOCAL_CTRL:  Gear: ");
  if(LOCAL_CTRL.Gear==NEUTRAL) display_L.print("N");
  else if(LOCAL_CTRL.Gear==DRIVE) display_L.print("D");
  else if(LOCAL_CTRL.Gear==REVERSE) display_L.print("R");
  else if(LOCAL_CTRL.Gear==PARK) display_L.print("P");
  else display_L.print("E");
  display_L.print("\nA:");
  print_float(display_L, LOCAL_CTRL.Angle);
  display_L.print(" S:");
  print_float(display_L, LOCAL_CTRL.Speed);
  display_L.print("\nREMOTE_CTRL: Gear: ");
  if(REMOTE_CTRL.Gear==NEUTRAL) display_L.print("N");
  else if(REMOTE_CTRL.Gear==DRIVE) display_L.print("D");
  else if(REMOTE_CTRL.Gear==REVERSE) display_L.print("R");
  else if(REMOTE_CTRL.Gear==PARK) display_L.print("P");
  else display_L.print("E");
  display_L.print("\nA:");
  print_float(display_L, REMOTE_CTRL.Angle);
  display_L.print(" S:");
  print_float(display_L, REMOTE_CTRL.Speed);
  display_L.print("\nEngage: ");
  display_L.print(REMOTE_CTRL.Engage);
  display_L.display();
  
  display_R.clearDisplay();
  if(LOCAL_STATUS.TurnIndicators==2&&led_timer) display_R.fillRect(0, 0, 16, 16, SH110X_WHITE);
  else display_R.drawRect(0, 0, 16, 16, SH110X_WHITE);
  if(LOCAL_STATUS.TurnIndicators==3&&led_timer) display_R.fillRect(128-16, 0, 16, 16, SH110X_WHITE);
  else display_R.drawRect(128-16, 0, 16, 16, SH110X_WHITE);
  display_R.display();
}