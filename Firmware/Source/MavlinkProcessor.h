#ifndef MAVLINK_PROCESSOR_H
#define MAVLINK_PROCESSOR_H

#include "GCS_MAVLink/GCS_MAVLink.h"
#include "FrSkySmartPort.h"

class MavlinkProcessor {
public:

	MavlinkProcessor();
	const unsigned long& getLastHeartbeat() const;
	void receiveTelemetry(Telemetry& gathered_telemetry);
	bool isConnected();
private:
	const int START = 1;
	const int MSG_RATE = 10; //Hz
	const uint8_t average_sample_count = 10;

	uint8_t is_connected;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_message_t msg;
	mavlink_status_t status;
	uint16_t heartbeat_count;
	int alive_led_pin;
	unsigned long connection_timer;
	unsigned long heartbeat_timer;
	unsigned long next_converter_heartbeat;


	void calculateVoltageAverage(uint16_t value);
	void calculateCurrentAverage(uint16_t value);
	void tryToConnectToAPM();

};
#endif
