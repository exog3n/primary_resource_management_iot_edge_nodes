#ifndef  FLOWMETER_H
#define  FLOWMETER_H

#include  <stdint.h>

class Flowmeter
{
  public:

    Flowmeter(uint8_t _fpin, uint16_t _tpl);

    void update();

    double getLitresPerMinute();
    double getLitresTotal();
 
    uint8_t getPin();

    void tick();

  private:

    uint16_t tpl  ;
    uint8_t fpin  ;
    
    unsigned long long ticks;

    unsigned long long lastCount_t;
    unsigned long long pduration  ;

    double current;
    double total  ;
};

#endif
