#ifndef Timer_h
#define Timer_h

class Timer
{
private:
  void (*_event)(void);
  bool _loop;
  unsigned long _period;
  unsigned long _timer;

public:
  Timer(void);

  void set(unsigned long period, void (*callback)(void));
  void set(unsigned long period, void (*callback)(void), bool uS);
  void start(void);
  void stop(void);
  void update(void);
};

#endif
