#ifndef MAVLINK_PROCESSOR_H
#define MAVLINK_PROCESSOR_H

//#include "mavlink/common/mavlink.h"
//#include "mavlink/ardupilotmega/mavlink.h"
#include "GCS_MAVLink/GCS_MAVLink.h"
class MavlinkProcessor {
public:
	struct MavlinkTelemetry {
	  //  MAVLINK_MSG_ID_HEARTBEAT
	  uint8_t    base_mode;
	  uint16_t   custom_mode;

	  // Message # 1  MAVLINK_MSG_ID_SYS_STATUS
	  uint16_t   battery_voltage;            // 1000 = 1V
	  int16_t    battery_current;            //  10 = 1A
	  int8_t     battery_remaining;          // Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery

	  // MAVLINK_MSG_ID_GPS_RAW_INT
	  uint8_t    gps_fixtype;                // 0= No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
	  uint8_t    gps_satellites_visible;     // number of visible satelites
	  int32_t    gps_latitude;               // 585522540;
	  int32_t    gps_longitude;              // 162344467;
	  int32_t    gps_altitude;               // 1000 = 1m
	  int32_t    gps_speed;                  // in cm/s
	  uint16_t   gps_hdop;                   // GPS HDOP horizontal dilution of position in cm

	  // MAVLINK_MSG_ID_VFR_HUD
	  uint32_t   groundspeed;                // Current ground speed in m/s
	  uint16_t   heading;                    // Current heading in degrees, in compass units (0..360, 0=north)
	  uint16_t   throttle;                   // Current throttle setting in integer percent, 0 to 100
	  int32_t    bar_altitude;               // 100 = 1m
	  int32_t    climb_rate;              // 100= 1m/s


	  // MAVLINK_MSG_ID_MISSION_CURRENT
	  uint16_t   mission_current_seq = 0;

	  // MAVLINK_MSG_ID_SCALED_PRESSURE
	  uint16_t   temperature = 0;

	  // MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT
	  uint16_t   wp_dist = 0;                // meters

	  // MAVLINK_MSG_ID_STATUSTEXT
	  uint16_t   ap_status_severity;
	  mavlink_statustext_t status_text;


	};

	MavlinkProcessor();
	const MavlinkTelemetry& getGatheredTelemetry() const;
	const unsigned long& getLastHeartbeat() const;
	void receiveTelemetry();
	bool isConnected();
private:
	const int START = 1;
	const int MSG_RATE = 10; //Hz
	const uint8_t average_sample_count = 10;

	MavlinkTelemetry gathered_telemetry;
	uint8_t is_connected;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_message_t msg;
	mavlink_status_t status;
	uint16_t heartbeat_count;
	int alive_led_pin;
	unsigned long connection_timer;
	unsigned long heartbeat_timer;

	void calculateVoltageAverage(uint16_t value);
	void calculateCurrentAverage(uint16_t value);
	void tryToConnectToAPM();
};
#endif
