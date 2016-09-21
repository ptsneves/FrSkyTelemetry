#ifndef FRSKY_PROCESSOR_H
#define FRSKY_PROCESSOR_H

#include <Arduino.h>
#include "FrSkySmartPort.h"
#include "MavlinkProcessor.h"
#include "SoftwareSerial/SoftwareSerial.h"

void frsky_send_text_message(char *msg);

class FrSkyProcessor {
public:

	enum SerialId {
		SOFT_SERIAL_PIN_2 = 2, SOFT_SERIAL_PIN_3 = 3, SOFT_SERIAL_PIN_4 = 4,
		SOFT_SERIAL_PIN_5 = 5, SOFT_SERIAL_PIN_6 = 6, SOFT_SERIAL_PIN_7 = 7,
		SOFT_SERIAL_PIN_8 = 8, SOFT_SERIAL_PIN_9 = 9, SOFT_SERIAL_PIN_10 = 10,
		SOFT_SERIAL_PIN_11 = 11, SOFT_SERIAL_PIN_12 = 12
	};

	FrSkyProcessor(const SerialId& mode, uint8_t fault_pin);
	~FrSkyProcessor();
	FrSkyProcessor(const FrSkyProcessor&) = delete;
	FrSkyProcessor& operator=(FrSkyProcessor&) = delete;
	void process(Telemetry& mav_telemetry, bool new_data);
private:
	void sendByte(uint8_t byte);
	void sendCrc();
	void sendPackage(uint8_t header, uint16_t id, uint32_t value);

	void sendSlowParameters(Telemetry& mav_telemetry);
	SlowParameters seq_last_slow_sent;
	SoftwareSerial softSerial;
	uint8_t fault_pin;

	short crc; // used for crc calc of frsky-packet
	uint8_t lastRx;
	uint8_t variometer_index;
	SerialId serial_id;

};
#endif
