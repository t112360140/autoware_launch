#include "Timer.h"
#include <Arduino.h>

Timer::Timer(void)
{
  _loop=false;
}

void Timer::set(unsigned long period, void (*callback)(void), bool uS){
  _timer=micros();
  _loop=true;
  _period=period;
  if(!uS) _period*=1000;

  _event=callback;
}

void Timer::set(unsigned long period, void (*callback)(void)){
  set(period, callback, false);
}

void Timer::update(void){
  if(_loop&&micros()-_timer>=_period){
    _event();
    
    _timer=micros();
  }
}

void Timer::start(void){
  _loop=true;
  _timer=micros();
}

void Timer::stop(void){
  _loop=false;
}
