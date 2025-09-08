#include <Arduino.h>

#include "SingleHumidistat.h"
#include "imath.h"

SingleHumidistat::SingleHumidistat(HumiditySensor *hs, const ConfigStore *cs,  etl::array<uint8_t, 2> pins_solenoid,
								   uint8_t pwmRes)
		: Humidistat(cs, hs, cs->HC_Kp, cs->HC_Ki, cs->HC_Kd, cs->HC_Kf, cs->dt, cs->S_lowValue, 1),
		  pins_solenoid{pins_solenoid[0], pins_solenoid[1]}, pwmRes(pwmRes) {}

void SingleHumidistat::update() {
	runCycle();

	// Actuate solenoids (convert normalised double CV to integer PWM value)
	analogWrite(pins_solenoid[0], static_cast<int>(cv * ipow(2, pwmRes)));
	analogWrite(pins_solenoid[1], static_cast<int>((pid.cvMin + 1 - cv) * ipow(2, pwmRes)));

#if defined(ARDUINO_TEENSYLC)
	// Output temperature to DAC (Teensy LC only, any supported sensor)
	double tempC = getTemperature();
	int dacValue = (int)((tempC - config::TEMP_DAC_MIN) * config::DAC_MAX / (config::TEMP_DAC_MAX - config::TEMP_DAC_MIN));
	if (dacValue < 0) dacValue = 0;
	if (dacValue > config::DAC_MAX) dacValue = config::DAC_MAX;
	analogWrite(config::PIN_DAC, dacValue);
#endif
}

void SingleHumidistat::updatePIDParameters() {
	pid.setGains(cs.HC_Kp, cs.HC_Ki, cs.HC_Kd, cs.HC_Kf, cs.dt);
	pid.cvMin = cs.S_lowValue;
}
