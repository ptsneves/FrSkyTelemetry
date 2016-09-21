//#include "WProgram.h"
#include "Arduino.h"
#include "FrSkyProcessor.h"
#include "MavlinkProcessor.h"

void __cxa_pure_virtual() {
	while(1){
		digitalWrite(13, LOW);
		delay(500);
		digitalWrite(13, HIGH);
		delay(2500);
	}
}

int main() {
	uint8_t led_pin = 13;
	init();
	FrSkyProcessor frsky_processor(FrSkyProcessor::SOFT_SERIAL_PIN_2, led_pin);
	MavlinkProcessor mavlink_processor {};
	Serial.begin(57600);
	pinMode(led_pin, OUTPUT);
	analogReference(DEFAULT);
	Telemetry gathered_telemetry = {};
	while(1){
		// Check MavLink communication
		mavlink_processor.receiveTelemetry(gathered_telemetry);
		// Check FrSky S.Port communication
		frsky_processor.process(gathered_telemetry, mavlink_processor.isConnected());
	}
	return 0; //unreachable
}
