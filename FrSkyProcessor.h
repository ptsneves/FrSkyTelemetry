#ifndef FRSKY_PROCESSOR_H
#define FRSKY_PROCESSOR_H

#include <Arduino.h>
#include "MavlinkProcessor.h"
#include "SoftwareSerial/SoftwareSerial.h"

void frsky_send_text_message(char *msg);

class FrSkyProcessor {
public:
	enum SerialId { SOFT_SERIAL_PIN_2 = 2, SOFT_SERIAL_PIN_3 = 3, SOFT_SERIAL_PIN_4 = 4, SOFT_SERIAL_PIN_5 = 5, SOFT_SERIAL_PIN_6 = 6, SOFT_SERIAL_PIN_7 = 7,
			SOFT_SERIAL_PIN_8 = 8, SOFT_SERIAL_PIN_9 = 9, SOFT_SERIAL_PIN_10 = 10, SOFT_SERIAL_PIN_11 = 11, SOFT_SERIAL_PIN_12 = 12 };
	FrSkyProcessor(const SerialId& mode, uint8_t fault_pin);
	~FrSkyProcessor();
	FrSkyProcessor(const FrSkyProcessor&) = delete;
	FrSkyProcessor& operator=(FrSkyProcessor&) = delete;
	void process(const MavlinkProcessor::MavlinkTelemetry& mav_telemetry, bool new_data);
private:
	void sendByte(uint8_t byte);
	void sendCrc();
	void sendPackage(uint8_t header, uint16_t id, uint32_t value);

	SoftwareSerial softSerial;
	uint8_t fault_pin;

	short crc; // used for crc calc of frsky-packet
	uint8_t lastRx;
	uint8_t gps_index;
	uint8_t variometer_index;
	uint8_t vfas_index;
	uint8_t misc_index;

	SerialId serial_id;

	// Frsky Sensor-ID to use.
	const uint8_t SENSOR_ID_VARIO = 0x00;
	const uint8_t SENSOR_ID_FAS = 0x22;
	const uint8_t SENSOR_ID_GPS = 0x1B; // ID of sensor. Must be something that is polled by FrSky RX
	const uint8_t SENSOR_ID_RPM = 0xE4;
	const uint8_t SENSOR_ID_SP2UH = 0x45;
	const uint8_t SENSOR_ID_SP2UR = 0xC6;
	// Frsky-specific
	const uint16_t START_STOP = 0x7e;
	const uint16_t DATA_FRAME = 0x10;


	//Frsky DATA ID's
	const uint16_t FR_ID_SPEED = 0x0830;
	const uint16_t FR_ID_VFAS = 0x0210;
	const uint16_t FR_ID_CURRENT = 0x0200;
	const uint16_t FR_ID_RPM = 0x050F;
	const uint16_t FR_ID_ALTITUDE= 0x0100;
	const uint16_t FR_ID_CUSTOM_MODE = 0x0600;
	const uint16_t FR_ID_ADC2 =  0xF103;
	const uint16_t FR_ID_LATLONG = 0x0800;
	const uint16_t FR_ID_CAP_USED = 0x0600;
	const uint16_t FR_ID_VARIO = 0x0110;
	const uint16_t FR_ID_CELLS = 0x0300;
	const uint16_t FR_ID_CELLS_LAST = 0x030F;
	const uint16_t FR_ID_HEADING = 0x0840;
	const uint16_t FR_ID_ACCX = 0x0700;
	const uint16_t FR_ID_ACCY = 0x0710;
	const uint16_t FR_ID_ACCZ = 0x0720;
	const uint16_t FR_ID_T1 = 0x0400;
	const uint16_t FR_ID_T2 = 0x0410;
	const uint16_t FR_ID_GPS_ALT = 0x0820;
	const uint16_t FR_GPS_SEC_ID = 0x1800;
	const uint16_t FR_ID_A3_FIRST = 0x0900; 	//A3_FIRST_ID
	const uint16_t FR_ID_A4_FIRST = 0x0910;;	//A4_FIRST_ID

	const uint16_t BAUD_RATE = 57600;
	const uint8_t sensor_1_2_max = 14;
	const uint8_t variometer_sensor_max = 1 +1;
	const uint8_t vfas_sensor_max = 1 + 1;
	const uint8_t gps_sensor_max = 3 + 1;
	const uint8_t misc_sensor_max = 5+ 1;

};
#endif
