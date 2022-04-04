#include <Arduino.h>

#include "valve.h"

Valve::Valve()
{
  
}
Valve::Valve(uint8_t _pin0, uint8_t _pin1 = 0, VALVE _type = STANDARD)
{
	if ( _type == STANDARD )
  {
    pinMode(_pin0, OUTPUT);
  }

  if ( _type == LATCHING )
  {
    pinMode(_pin0, OUTPUT);
    pinMode(_pin1, OUTPUT);
  }

	this->pin0 = _pin0;
	this->pin1 = _pin1;
	this->type = _type;

	this->invert = false;
	this->pulseWidth = 5;

	this->current = RANDOM;
}

void Valve::setInverted(bool _invert = true)
{
	this->invert = _invert;
}

void Valve::setPulse(uint8_t _pulse)
{
	this->pulseWidth = _pulse;
}

void Valve::open()
{
	if ( this->type == STANDARD )
	{
		// Normally, we'd want a HIGH to open the valve.
		// But we need to make sure if it's an inverted valve:
		//
		digitalWrite(this->pin0, (!this->invert & HIGH));
	}

	if ( this->type == LATCHING )
	{
		// For safety let's close both ports
		// before any other ops:
		//
		digitalWrite(this->pin0, 0);
		digitalWrite(this->pin1, 0);

		// Trigger the correct pin (depending on whether
		// it's an inverting valve or not) for the specified
		// amount of time (in millisecs):
		//
		if ( !this->invert )
			digitalWrite(this->pin0, HIGH);
		else
			digitalWrite(this->pin1, HIGH);

		delay(this->pulseWidth);

		// Finally, we're closing both
		// GPIO pins:
		//
		digitalWrite(this->pin0, 0);
		digitalWrite(this->pin1, 0);
	}

	this->current = OPENED;
}

void Valve::shut()
{
	if ( this->type == STANDARD )
	{
		// Normally, we'd want a LOW to shut the valve.
		// But we need to make sure if it's an inverted valve first:
		//
		digitalWrite(this->pin0, (!this->invert & 0));
	}

	if ( this->type == LATCHING )
	{
		// For safety let's close both ports
		// before any other ops:
		//
		digitalWrite(this->pin0, 0);
		digitalWrite(this->pin1, 0);

		// Trigger the correct pin (depending on whether
		// it's an inverting valve or not) for the specified
		// amount of time (in millisecs):
		//
		if ( !this->invert )
			digitalWrite(this->pin1, HIGH);
		else
			digitalWrite(this->pin0, HIGH);

		delay(this->pulseWidth);

		// Finally, we're closing both
		// GPIO pins:
		//
		digitalWrite(this->pin0, 0);
		digitalWrite(this->pin1, 0);
	}
	this->current = CLOSED;
}

STATE Valve::getStatus()
{
	return this->current;
}
