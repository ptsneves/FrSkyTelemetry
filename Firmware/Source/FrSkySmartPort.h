/*
 * FrSkySmartPort.h
 *
 *  Created on: Jan 13, 2016
 *      Author: pneves
 */

#ifndef FRSKYSMARTPORT_H_
#define FRSKYSMARTPORT_H_

enum SlowParameters {
	BASE_MODE = 0,
	CUSTOM_MODE,
	GPS_FIX_TYPE_AND_SAT_VISIBLE,
	HDOP_EX,
	SYSTEM_STATUS,
	MAV_TYPE,
	BATTERY_REMAINING,
	INVALID_PARAMETER
};

struct Telemetry {
	//  MAVLINK_MSG_ID_HEARTBEAT
	uint8_t base_mode;
	uint16_t custom_mode;
	uint8_t system_status;
	uint8_t mav_type;

	// Message # 1  MAVLINK_MSG_ID_SYS_STATUS
	uint16_t battery_voltage; // 1000 = 1V
	int16_t battery_current; //  10 = 1A
	int8_t battery_remaining; // Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery

	// MAVLINK_MSG_ID_GPS_RAW_INT
	uint8_t gps_fixtype; // 0= No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
	uint8_t gps_satellites_visible; // number of visible satelites
	int32_t gps_latitude; // 585522540;
	int32_t gps_longitude; // 162344467;

	int32_t gps_base_latitude;
	int32_t gps_base_longitude;
	uint16_t base_heading;

	int32_t gps_altitude; // 1000 = 1m
	int32_t gps_speed; // in cm/s
	uint16_t gps_hdop; // GPS HDOP horizontal dilution of position in cm

	// MAVLINK_MSG_ID_VFR_HUD
	uint32_t groundspeed; // Current ground speed in m/s
	uint16_t heading; // Current heading in degrees, in compass units (0..360, 0=north)
	uint16_t throttle; // Current throttle setting in integer percent, 0 to 100
	int32_t bar_altitude; // 100 = 1m
	int32_t climb_rate; // 100= 1m/s

	//MAVLINK_MSG_ID_ATTITUDE
	int32_t roll;
	int32_t pitch;
	int32_t yaw;

	// MAVLINK_MSG_ID_MISSION_CURRENT
	uint16_t mission_current_seq = 0;

	// MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT
	uint16_t wp_dist = 0; // meters

	// MAVLINK_MSG_ID_STATUSTEXT
	uint16_t ap_status_severity;
	char text[50];

	uint16_t slow_changed_parameter; //bitfield

};

enum FrSkySmartPortProtocolConstants {
	START_STOP = 0x7e,
	DATA_FRAME = 0x10,
	BAUD_RATE = 57600
};

enum FrSkySensorId {
	FR_ID_SPEED = 0x0830,
	FR_ID_VFAS = 0x0210,
	FR_ID_CURRENT = 0x0200,
	FR_ID_RPM = 0x050F,
	FR_ID_TEXT = FR_ID_RPM,
	FR_ID_ALTITUDE = 0x0100,
	FR_ID_CUSTOM_MODE = 0x0600,
	FR_ID_HDOP = 0xF103,
	FR_ID_ADC2 = 0xF103,
	FR_ID_LATLONG = 0x0800,
	FR_ID_CAP_USED = 0x0600,
	FR_ID_VARIO = 0x0110,
	FR_ID_CELLS = 0x0300,
	FR_ID_CELLS_LAST = 0x030F,
	FR_ID_HEADING = 0x0840,
	FR_ID_ACCX = 0x0700,
	FR_ID_BAT_REMAINING = 0x0700,
	FR_ID_ACCY = 0x0710,
	FR_ID_ARMED = 0x0710,
	FR_ID_ACCZ = 0x0720,
	FR_ID_GPS_STATUS = 0x0400,
	FR_ID_T1 = 0x0400,
	FR_ID_T2 = 0x0410,
	FR_ID_GPS_ALT = 0x0820,
	FR_GPS_SEC_ID = 0x1800,
	FR_ID_A3_FIRST = 0x0900, //A3_FIRST_ID
	FR_ID_A4_FIRST = 0x0910, //A4_FIRST_ID
	FR_ID_ROLL = 0x0A3E, //AIR_SPEED_FIRST_ID
	FR_ID_PITCH = 0x0A3D, //FR_ID_ACCZ
	FR_ID_BASE_MODE = 0x0A4C,
	FR_ID_SYSTEM_STATUS = 0x0A5B,
	FR_ID_MAV_TYPE = 0x0A6B,
	FR_ID_HDOP_EX = 0x0A7B
};

enum FrSkySensors {
	// Frsky Sensor-ID to use.
	SENSOR_ID_VARIO = 0x00,
	SENSOR_ID_FAS = 0x22,
	SENSOR_ID_GPS = 0x1B, // ID of sensor. Must be something that is polled by FrSky RX
	SENSOR_ID_RPM = 0xE4,
	SENSOR_ID_SP2UH = 0x45,
	SENSOR_ID_SP2UR = 0xC6
};

#endif /* FRSKYSMARTPORT_H_ */
