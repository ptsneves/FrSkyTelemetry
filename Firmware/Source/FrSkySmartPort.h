/*
 * FrSkySmartPort.h
 *
 *  Created on: Jan 13, 2016
 *      Author: pneves
 */

#ifndef FRSKYSMARTPORT_H_
#define FRSKYSMARTPORT_H_

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
	FR_ID_ALTITUDE= 0x0100,
	FR_ID_CUSTOM_MODE = 0x0600,
	FR_ID_ADC2 =  0xF103,
	FR_ID_LATLONG = 0x0800,
	FR_ID_CAP_USED = 0x0600,
	FR_ID_VARIO = 0x0110,
	FR_ID_CELLS = 0x0300,
	FR_ID_CELLS_LAST = 0x030F,
	FR_ID_HEADING = 0x0840,
	FR_ID_ACCX = 0x0700,
	FR_ID_ACCY = 0x0710,
	FR_ID_ACCZ = 0x0720,
	FR_ID_T1 = 0x0400,
	FR_ID_T2 = 0x0410,
	FR_ID_GPS_ALT = 0x0820,
	FR_GPS_SEC_ID = 0x1800,
	FR_ID_A3_FIRST = 0x0900, 	//A3_FIRST_ID
	FR_ID_A4_FIRST = 0x0910	//A4_FIRST_ID
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
