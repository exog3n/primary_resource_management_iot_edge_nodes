#include "flowmeter.h"
#include <Arduino.h>
#include <stdint.h>

Flowmeter::Flowmeter(uint8_t _fpin, uint16_t _tpl)
{
  pinMode(this->fpin, INPUT_PULLUP);
  this->fpin = _fpin;

  this->tpl = _tpl;

  this->lastCount_t = 0;
  this->pduration = 0  ;

  this->current = 0.0;
  this->total = 0.0  ;

  this->ticks = 0;
}
uint8_t Flowmeter::getPin()
{
  return this->fpin;
}
void Flowmeter::tick()
{
  this->ticks += 1;
}
void Flowmeter::update()
{
  if (this->ticks > 0)
  {
    this->current = this->ticks * (1.0 / this->tpl);

    this->total  += current;

    this->pduration = millis() - lastCount_t;
    this->lastCount_t = millis();

    this->ticks = 0;
  }
  return;
}

double Flowmeter::getLitresPerMinute()
{
  // If duration is < 1 it means that there is no measurement yet:
  //
  if (!this->pduration < 1)
  {
    return -1;
  }

  // Get the duration of the last measurement.
  // This is needed to calculate the flow rate:
  //
  unsigned long long pduration = this->pduration;
  this->pduration = 0;
  return ( (this->current * 60000) / pduration);
}
double Flowmeter::getLitresTotal()
{
  double total = this->total;
  this->total  = 0.0;
  return total ;
}
