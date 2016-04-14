/*
The code has contributions from
Rolf Blomgren http://diydrones.com/profile/RolfBlomgren; http://diydrones.com/forum/topics/amp-to-frsky-x8r-sport-converter?xg_source=activity&id=705844%3ATopic%3A1539556;
Pawelky http://www.rcgroups.com/forums/member.php?u=393936; http://www.rcgroups.com/forums/showthread.php?t=2245978
Paulo Neves  https://www.airborneprojects.com/services

APM2.5 Mavlink to FrSky X8R SPort interface using Teensy 3.1  http://www.pjrc.com/teensy/index.html
based on ideas found here http://code.google.com/p/telemetry-convert/
******************************************************
Cut board on the backside to separate Vin from VUSB

Connection on Teensy 3.1:
SPort S --> TX1
SPort + --> Vin
SPort  - --> GND

APM Telemetry DF13-5  Pin 2 --> RX2
APM Telemetry DF13-5  Pin 3 --> TX2
APM Telemetry DF13-5  Pin 5 --> GND

Analog input  --> A0 (pin14) on Teensy 3.1 ( max 3.3 V )


This is the data we send to FrSky, you can change this to have your own
set of data
******************************************************
Data transmitted to FrSky Taranis:
Cell           ( Voltage of Cell=Cells/4. [V] This is my LiPo pack 4S ) 
Cells         ( Voltage from LiPo [V] )
A2             ( Analog voltage from input A0 on Teensy 3.1 )
Alt             ( Altitude from baro.  [m] )
GAlt          ( Altitude from GPS   [m])
HDG         ( Compass heading  [deg])
Rpm         ( Throttle when ARMED [%] )
AccX         ( AccX m/s ? )
AccY         ( AccY m/s ? )
AccZ         ( AccZ m/s ? )
VSpd        ( Vertical speed [m/s] )
Speed      ( Ground speed from GPS,  [km/h] )
T1            ( GPS status = ap_sat_visible*10) + ap_fixtype )
T2            ( ARMED=1, DISARMED=0 )
Vfas          ( same as Cells )
Longitud    
Latitud
Dist          ( Will be calculated by FrSky Taranis as the distance from first received lat/long = Home Position

******************************************************

*/

#include "MavlinkProcessor.h"
#include "FrSkyProcessor.h"
#include "Arduino.h"
#define MavLinkSerial Serial

MavlinkProcessor::MavlinkProcessor() :
	gathered_telemetry{},
	is_connected{false},
	buf{0},
	msg{0},
	status{0},
	heartbeat_count{0},
	alive_led_pin{13},
	connection_timer{0},
	heartbeat_timer{millis()} {
		MavLinkSerial.begin(57600);

}

const MavlinkProcessor::MavlinkTelemetry& MavlinkProcessor::getGatheredTelemetry() const {
	return gathered_telemetry;
}

const unsigned long& MavlinkProcessor::getLastHeartbeat() const {
	return heartbeat_timer;
}

void MavlinkProcessor::receiveTelemetry() {
	uint32_t next_timeout = millis() + 6;
	tryToConnectToAPM();
	while(MavLinkSerial.available() && millis() < next_timeout){
		uint8_t c = MavLinkSerial.read();
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			switch(msg.msgid)	{
			case MAVLINK_MSG_ID_HEARTBEAT:  // 0
				if (msg.sysid == 1) {
					gathered_telemetry.base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
					gathered_telemetry.custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
					connection_timer = millis();
					if(!is_connected); {
						heartbeat_count++;
						if((heartbeat_count) > 10) {        // If  received > 10 heartbeats from MavLink then we are connected
							is_connected = true;
							heartbeat_count = 0;
							digitalWrite(alive_led_pin, HIGH);      // LED will be ON when connected to MavLink, else it will slowly blink
						}
					}
				}
				break;
			case MAVLINK_MSG_ID_SYS_STATUS :   // 1
				gathered_telemetry.battery_voltage = mavlink_msg_sys_status_get_voltage_battery(&msg) / 10;  //
				gathered_telemetry.battery_current = mavlink_msg_sys_status_get_current_battery(&msg) / 10;     //
				gathered_telemetry.battery_remaining = mavlink_msg_sys_status_get_battery_remaining(&msg);
				break;
			case MAVLINK_MSG_ID_GPS_RAW_INT:   // 24
				gathered_telemetry.gps_fixtype = mavlink_msg_gps_raw_int_get_fix_type(&msg);// 0 = No GPS, 1 =No Fix, 2 = 2D Fix, 3 = 3D Fix
				gathered_telemetry.gps_satellites_visible =  mavlink_msg_gps_raw_int_get_satellites_visible(&msg);          // numbers of visible satelites
				if(gathered_telemetry.gps_fixtype >= 3)  {
					gathered_telemetry.gps_hdop = mavlink_msg_gps_raw_int_get_eph(&msg);  // hdop * 100
					gathered_telemetry.gps_latitude = mavlink_msg_gps_raw_int_get_lat(&msg);
					gathered_telemetry.gps_longitude = mavlink_msg_gps_raw_int_get_lon(&msg);
					gathered_telemetry.gps_altitude = mavlink_msg_gps_raw_int_get_alt(&msg);    // 1m =1000
					gathered_telemetry.gps_speed = mavlink_msg_gps_raw_int_get_vel(&msg);
				}
				else {
					gathered_telemetry.gps_hdop = 9999;
					gathered_telemetry.gps_latitude = 0L;
					gathered_telemetry.gps_longitude = 0l;
					gathered_telemetry.gps_altitude = 0L;
					gathered_telemetry.gps_speed = 0L;
				}
				break;
			case MAVLINK_MSG_ID_VFR_HUD:   //  74
				gathered_telemetry.groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);      // 100 = 1m/s
				gathered_telemetry.heading = mavlink_msg_vfr_hud_get_heading(&msg) ;     // 100 = 100 deg
				gathered_telemetry.throttle = mavlink_msg_vfr_hud_get_throttle(&msg);        //  100 = 100%
				gathered_telemetry.bar_altitude = mavlink_msg_vfr_hud_get_alt(&msg);        //  m
				gathered_telemetry.climb_rate = mavlink_msg_vfr_hud_get_climb(&msg) * 100;        //  m/s
				break;
			case MAVLINK_MSG_ID_STATUSTEXT:
				mavlink_msg_statustext_decode(&msg, &gathered_telemetry.status_text);
				frsky_send_text_message(gathered_telemetry.status_text.text);
				break;
			default:
				break;
			}
		}
	}
}

void MavlinkProcessor::tryToConnectToAPM() {
	uint16_t len = 0;
	if (millis() - heartbeat_timer > 1500UL) {
		heartbeat_timer = millis();
		if (!is_connected) {    // Start requesting data streams from MavLink
			digitalWrite(alive_led_pin,HIGH);
			mavlink_msg_request_data_stream_pack(0xFF, 0xBE, &msg, 1, 1, MAV_DATA_STREAM_EXTENDED_STATUS,
					MSG_RATE, START);
			len = mavlink_msg_to_send_buffer(buf, &msg);
			MavLinkSerial.write(buf, len);
			delay(10);
			mavlink_msg_request_data_stream_pack(0xFF, 0xBE, &msg, 1, 1, MAV_DATA_STREAM_EXTRA2,
					MSG_RATE, START);
			len = mavlink_msg_to_send_buffer(buf, &msg);
			MavLinkSerial.write(buf, len);
			delay(10);
			mavlink_msg_request_data_stream_pack(0xFF, 0xBE, &msg, 1, 1, MAV_DATA_STREAM_RAW_SENSORS,
					MSG_RATE, START);
			len = mavlink_msg_to_send_buffer(buf, &msg);
			MavLinkSerial.write(buf,len);
			digitalWrite(alive_led_pin, LOW);
		}
	}

	if((millis() - connection_timer) > 1500)  {   // if no HEARTBEAT from APM  in 1.5s then we are not connected
		is_connected = false;
		heartbeat_count = 0;
	}
}

bool MavlinkProcessor::isConnected() {
	return is_connected;
}
