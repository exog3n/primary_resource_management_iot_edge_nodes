#ifndef VALVE_H
#define VALVE_H

#include <stdint.h>

enum VALVE
{
  STANDARD = 0,
  LATCHING,
};

enum STATE
{
	RANDOM = -1,
	CLOSED =  0,
	OPENED =  1,
};

class Valve
{
public:

  Valve();
 	Valve(uint8_t _pin0, uint8_t _pin1 = 0, VALVE _type = STANDARD);

	void setInverted(bool invert = true);
	void setPulse(uint8_t pulse) ;

 	void open();
 	void shut();

 	STATE getStatus();

private:

	uint8_t pin0 ;
	uint8_t pin1 ;
	VALVE   type ;

	STATE current;

	bool invert;
	uint8_t pulseWidth;
};

#endif
