#include "board.h"
#include "mw.h"
#include "baseflight_mavlink.h"

static bool mavlink_send_paralist;
static bool mavlink_send_mission;

bool BlockProtocolChange;

static uint32_t ControlAndSensorsPresent = 35843;
static uint8_t  system_type;

//static uint8_t  mavlink_send_mission = 0;
//static uint8_t  mavlink_get_mission = 0;
//static uint16_t mission_max = 0;

typedef struct {
	float p_lat;
	float p_long;
	float p_alt;
	float yaw;
	float radius;
	float wait;
	float orbit;
	char name[128];
	char command[128];
	uint8_t type;
} WayPoint;

#define MAX_WAYPOINTS 5
volatile WayPoint WayPoints[MAX_WAYPOINTS];

void baseflight_mavlink_init(void)
{
    switch(cfg.mixerConfiguration)                    // Set system_type here
    {
    case MULTITYPE_TRI:
        system_type = MAV_TYPE_TRICOPTER;
        break;
    case MULTITYPE_QUADP:
    case MULTITYPE_QUADX:
    case MULTITYPE_Y4:
    case MULTITYPE_VTAIL4:
        system_type = MAV_TYPE_QUADROTOR;
        break;
    case MULTITYPE_Y6:
    case MULTITYPE_HEX6:
    case MULTITYPE_HEX6X:
        system_type = MAV_TYPE_HEXAROTOR;
        break;
    case MULTITYPE_OCTOX8:
    case MULTITYPE_OCTOFLATP:
    case MULTITYPE_OCTOFLATX:
        system_type = MAV_TYPE_OCTOROTOR;
        break;
    case MULTITYPE_FLYING_WING:
    case MULTITYPE_AIRPLANE:
        system_type = MAV_TYPE_FIXED_WING;
        break;
    case MULTITYPE_HELI_120_CCPM:
    case MULTITYPE_HELI_90_DEG:
        system_type = MAV_TYPE_HELICOPTER;
        break;
    default:
        system_type = MAV_TYPE_GENERIC;
        break;
    }
/*
    onboard_control_sensors_present Bitmask
    fedcba9876543210
    1000110000000011    For all   = 35843
    0001000000000100    With Mag  = 4100
    0010000000001000    With Baro = 8200
    0100000000100000    With GPS  = 16416
    0000001111111111
*/
    if (sensors(SENSOR_MAG))  ControlAndSensorsPresent |=  4100;
    if (sensors(SENSOR_BARO)) ControlAndSensorsPresent |=  8200;
    if (sensors(SENSOR_GPS))  ControlAndSensorsPresent |= 16416;
    reset_mavlink();
}

void reset_mavlink(void)
{
    Currentprotocol = PROTOCOL_AUTOSENSE;                                // Set primary Protocol to unknown/autosensing
    baseflight_mavlink_send_paramlist(true);                             // Stop sending parameterlist, if it was sending during arm/disarm
    mavlink_send_paralist = false;
    mavlink_send_mission  = false;  
}

bool baseflight_mavlink_send_1Hzheartbeat(void)                          // That mother is running at 1Hz and schedules/collects eeprom writes
{
    mavlink_message_t msg2;
    static uint32_t   LastHeartbeat;
    uint8_t           autopilot_type = MAV_AUTOPILOT_ARDUPILOTMEGA;      // uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
	  uint8_t           system_mode    = 0;                                // Is set below
	  uint32_t          custom_mode    = 0;
	  uint8_t           system_state   = MAV_STATE_STANDBY;

    if ((currentTimeMS - LastHeartbeat) < 1000) return false;
    LastHeartbeat = currentTimeMS;

//  Set this here if Automission: MAV_MODE_STABILIZE_DISARMED
    if (f.ANGLE_MODE || f.HORIZON_MODE) system_mode = MAV_MODE_STABILIZE_DISARMED;
     else system_mode = MAV_MODE_MANUAL_DISARMED;
    if(f.ARMED)
    {
        system_mode |= 128;                                              // Set the Armed bit here if necessary
        system_state = MAV_STATE_ACTIVE;
    }
    mavlink_msg_heartbeat_pack(1, 200, &msg2, system_type, autopilot_type, system_mode, custom_mode, system_state);
    baseflight_mavlink_send_message(&msg2);
    return true;
}

void baseflight_mavlink_send_message(mavlink_message_t* msg)
{
    uint8_t  buf[MAVLINK_MAX_PACKET_LEN];
	  uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
	  uint16_t i;
	  for (i = 0; i < len; i++) uartWrite(buf[i]);
}

bool baseflight_mavlink_receive(char new)
{
    static mavlink_message_t msg;
	  static mavlink_status_t  status;
	  if (mavlink_parse_char(0, new, &msg, &status))
    {
        baseflight_mavlink_handleMessage(&msg);
        return true;
    }
    else return false;
}

void baseflight_mavlink_handleMessage(mavlink_message_t *msg)
{
//    mavlink_message_t      msg2;
    switch (msg->msgid)
    {
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        if (!mavlink_send_paralist)
        {
          baseflight_mavlink_send_paramlist(true); // Just reset function to send from the beginning
          mavlink_send_paralist = true;        // Only initiate paralist send here, and not already sending
        }
			  break;
        
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
    		if (!mavlink_send_mission) mavlink_send_mission = true; // Start sending the list, if it isn't already running
			  break;
        
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        {
            mavlink_param_request_read_t packet;
            mavlink_msg_param_request_read_decode(msg, &packet);
            baseflight_mavlink_send_singleparam(packet.param_index);
            break;
        }
		case MAVLINK_MSG_ID_PARAM_SET:
        {
            mavlink_param_set_t packet;
			      mavlink_msg_param_set_decode(msg, &packet);
            if(baseflight_mavlink_set_param(&packet)) ScheduleEEPROMwriteMS = currentTimeMS + 500; // Collect some EEPROMWRITES BEFORE ACTUALLY DOING IT
      	    break;
        }
		default:
			  break;
    }
}

void baseflight_mavlink_send_updates(void)                                      // That's a bad mother here :)
{
    static uint32_t   Timer100Hz;
    static uint8_t    HudPackCnt, AttiPackCnt, GPSPackCnt, RCPackCnt, ParaLstCnt, StatuspackCnt, PressPackCnt;
    uint16_t          voltage = 0;
    mavlink_message_t msg2;
    bool              PacketSent;                                               // Avoid too much stuff in one Action
    int16_t           tmp1;

//  NOTE: THE HZ NUMBERS ARE WISHFUL THINKING, BECAUSE IT IS ENSURED THAT ONLY ONE PACKET IS SENT PER RUN
//  SO THE ACTUAL HZ WILL DEGRADE, CHECK WITH GCS FOR REAL DATARATES

    PacketSent = baseflight_mavlink_send_1Hzheartbeat();                        // Does internal 1Hz Timer returns true when done
    if ((currentTime - Timer100Hz) >= 10000)                                    // 100Hz Timebase for mavlink because it's slow anyways
	  {
        Timer100Hz = currentTime;
        HudPackCnt++; AttiPackCnt++; GPSPackCnt++; RCPackCnt++; ParaLstCnt++;   // INCREASE TIMERS
        StatuspackCnt++; PressPackCnt++;

        if (PressPackCnt >= 200 && !PacketSent)                                 // 0.5Hz for Pressure Pack
        {
            PressPackCnt = 0;
            PacketSent   = true;
            mavlink_msg_scaled_pressure_pack(
                1, 200, &msg2, currentTimeMS, ActualPressure * 0.01f, 0, telemTemperature1 * 100);
            baseflight_mavlink_send_message(&msg2);
        }

        if (RCPackCnt >= 47 && !PacketSent)                                     // 2Hz for RC
        {
            RCPackCnt  = 0;
            PacketSent = true;
            mavlink_msg_rc_channels_raw_pack(
                1, 200, &msg2, currentTimeMS, 0, rcData[0], rcData[1], rcData[3],
                rcData[2], rcData[4], rcData[5], rcData[6], rcData[7], rssi);
            baseflight_mavlink_send_message(&msg2);
        }

        if (StatuspackCnt >= 48 && !PacketSent)                                 // 2Hz for Status
        {
            StatuspackCnt = 0;
            PacketSent    = true;
            if (FEATURE_VBAT) voltage = (uint16_t)vbat * 100;                   // in mV
            mavlink_msg_sys_status_pack(
                1, 200, &msg2, ControlAndSensorsPresent, ControlAndSensorsPresent,
                ControlAndSensorsPresent & 1023, 0, voltage, -1, -1, 0, 0, 0, 0, 0, 0);
            baseflight_mavlink_send_message(&msg2);
        }

        if (sensors(SENSOR_GPS) && GPSPackCnt >= 49 && !PacketSent)             // 2Hz for GPS
        {
            GPSPackCnt = 0;
            PacketSent = true;
            if (f.GPS_FIX) tmp1 = 3;                                            // Report 3Dfix if any fix
             else tmp1 = 0;
	          mavlink_msg_gps_raw_int_pack(
                1, 200, &msg2, currentTime, (uint8_t)tmp1, Real_GPS_coord[LAT], Real_GPS_coord[LON], GPS_altitude * 1000,
                65535, 65535, GPS_speed, constrain(GPS_ground_course * 10, 0, 35999), GPS_numSat);
            baseflight_mavlink_send_message(&msg2);
        }
        
        if (HudPackCnt >= 10 && !PacketSent)                                    // 10Hz for HUD
        {
            HudPackCnt = 0;
            PacketSent = true;
            tmp1 = 0;
            if (sensors(SENSOR_MAG))
            {
                tmp1 = heading;
                if (tmp1 < 0) tmp1 = tmp1 + 360;                                // heading in degrees, in compass units (0..360, 0=north)
            }
            mavlink_msg_vfr_hud_pack(
                1, 200, &msg2, 0, (float)GPS_speed * 0.01f, tmp1,
                ((int32_t)(rcCommand[THROTTLE] - cfg.esc_min) * 100)/(cfg.esc_max - cfg.esc_min),
                EstAlt * 0.01f, vario * 0.01f);
            baseflight_mavlink_send_message(&msg2);
        }
        
        if (ParaLstCnt >= 10 && !PacketSent)                                    // 10Hz for Parameterlist transmission
        {
            ParaLstCnt = 0;
            if (mavlink_send_paralist)
            {
                PacketSent = true;
                mavlink_send_paralist = !baseflight_mavlink_send_paramlist(false);  // baseflight_mavlink_send_param_lst is true when done
            }
        }

        
//        MAVLINK_MSG_ID_MISSION_REQUEST_LIST
//        static bool mavlink_send_mission;
//        mavlink_msg_MISSION
        
        if (AttiPackCnt >= 3 && !PacketSent)                                    // 30Hz for Attitude
        {
            AttiPackCnt = 0;
            mavlink_msg_attitude_pack(
                1, 200, &msg2, currentTimeMS, angle[0] * RADX10, -angle[1] * RADX10,
                heading * RADX, 0.0f, 0.0f, 0.0f);
	          baseflight_mavlink_send_message(&msg2);
        }
	  }
}

/*
void baseflight_mavlink_handleMessage (mavlink_message_t *msg)
{
    mavlink_message_t      msg2;

    switch (msg->msgid)
    {
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:			//printf("## MSG_ID: PARAM_REQUEST ##\r\n");
        if (mavlink_send_parameter == 0)
        {
				    mavlink_send_parameter = 1;
			  }
			  break;
		case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:   //printf("## MSG_ID: MISSION_REQUEST_LIST ##\r\n");
    		if (mavlink_send_mission == 0)
            mavlink_send_mission = 1;
			  break;
		case MAVLINK_MSG_ID_COMMAND_LONG:
        {
            mavlink_command_long_t packet;          
            mavlink_msg_command_long_decode(msg, &packet);
			      if (packet.command == MAV_CMD_PREFLIGHT_STORAGE && packet.param1 == 1.0)
                cliSave("");
        }
        break;
    case MAVLINK_MSG_ID_MISSION_COUNT:          //printf("## MSG_ID: MISSION_COUNT ##\r\n");
        if (mavlink_get_mission == 0)
        {
            mavlink_mission_count_t packet;
				    mavlink_msg_mission_count_decode(msg, &packet);
    				mission_max = packet.count;
    				mavlink_msg_mission_request_pack(1, 200, &msg2, 1, 200, 0);
		    		baseflight_mavlink_send_message(&msg2);
			  }
        break;
		case MAVLINK_MSG_ID_MISSION_ITEM:
        {
            mavlink_mission_item_t packet;
			      mavlink_msg_mission_item_decode(msg, &packet); //printf("RECEIVED MISSION_ITEM: %i/%i: %f, %f, %f (%i)\n", packet.seq, mission_max, packet.x, packet.y, packet.z, packet.frame);
            if (packet.seq < mission_max - 1)
            {
				        mavlink_msg_mission_request_pack(127, 0, &msg2, 1, 200, packet.seq + 1);
				        baseflight_mavlink_send_message(&msg2);
			      }
            else
            {
			    	    mavlink_msg_mission_ack_pack(127, 0, &msg2, 1, 200, 15);
				        baseflight_mavlink_send_message(&msg2);
			      }
            
			      if (packet.seq > 0) packet.seq = packet.seq - 1;

            switch (packet.command)			//printf("getting WP(%i): %f, %f\n", packet.seq, packet.x, packet.y);
            {
				        case MAV_CMD_NAV_WAYPOINT:
					          strcpy((char *)WayPoints[1 + packet.seq].command, "WAYPOINT");
					          break;
        				case MAV_CMD_NAV_LOITER_UNLIM:
				          	strcpy((char *)WayPoints[1 + packet.seq].command, "LOITER_UNLIM");
					          break;
        				case MAV_CMD_NAV_LOITER_TURNS:
				          	strcpy((char *)WayPoints[1 + packet.seq].command, "LOITER_TURNS");
					          break;
        				case MAV_CMD_NAV_LOITER_TIME:
				          	strcpy((char *)WayPoints[1 + packet.seq].command, "LOITER_TIME");
          					break;
        				case MAV_CMD_NAV_RETURN_TO_LAUNCH:
          					strcpy((char *)WayPoints[1 + packet.seq].command, "RTL");
					          break;
        				case MAV_CMD_NAV_LAND:
          					strcpy((char *)WayPoints[1 + packet.seq].command, "LAND");
					          break;
        				case MAV_CMD_NAV_TAKEOFF:
          					strcpy((char *)WayPoints[1 + packet.seq].command, "TAKEOFF");
					          break;
        				default:
				          	sprintf((char *)WayPoints[1 + packet.seq].command, "CMD:%i", packet.command);
					          break;
      			}

			      if (packet.x == 0.0) packet.x = 0.00001;
      			if (packet.y == 0.0) packet.y = 0.00001;
      			if (packet.z == 0.0) packet.z = 0.00001;

			      WayPoints[1 + packet.seq].p_lat  = packet.x;
			      WayPoints[1 + packet.seq].p_long = packet.y;
			      WayPoints[1 + packet.seq].p_alt  = packet.z;
			      WayPoints[1 + packet.seq].yaw    = packet.param4;
			      sprintf((char *)WayPoints[1 + packet.seq].name, "WP%i", packet.seq + 1);

			      WayPoints[1 + packet.seq + 1].p_lat = 0.0;
			      WayPoints[1 + packet.seq + 1].p_long = 0.0;
			      WayPoints[1 + packet.seq + 1].p_alt = 0.0;
			      WayPoints[1 + packet.seq + 1].yaw = 0.0;
			      strcpy((char *)WayPoints[1 + packet.seq + 1].name, "");
			      strcpy((char *)WayPoints[1 + packet.seq + 1].command, "");
			  break;
		    }
		case MAVLINK_MSG_ID_PARAM_SET:
        {
            mavlink_param_set_t packet;
			      mavlink_msg_param_set_decode(msg, &packet); //printf("## MSG_VALUE: %s %f ##\r\n", packet.param_id, packet.param_value);
      			baseflight_mavlink_set_param(&packet);
      	break;
		    }
		default:	//printf("## MSG_ID: %i ##\r\n", msg->msgid);
				break;
	}
}

 * @brief Pack a scaled_pressure message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param press_abs Absolute pressure (hectopascal)
 * @param press_diff Differential pressure 1 (hectopascal)
 * @param temperature Temperature measurement (0.01 degrees celsius)
 * @return length of the message in bytes (excluding serial stream start sign)
uint16_t mavlink_msg_scaled_pressure_pack(
                                          uint8_t system_id,
                                          uint8_t component_id,
                                          mavlink_message_t* msg,
                                          uint32_t time_boot_ms,
                                          float press_abs,
                                          float press_diff,
                                          int16_t temperature)

Packet INFO
 * @brief Pack a heartbeat message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot Autopilot type / class. defined in MAV_AUTOPILOT ENUM
 * @param base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
 * @param custom_mode A bitfield for use for autopilot-specific flags.
 * @param system_status System status flag, see MAV_STATE ENUM
 * @return length of the message in bytes (excluding serial stream start sign)

uint16_t mavlink_msg_heartbeat_pack(uint8_t system_id,
                                    uint8_t component_id,
                                    mavlink_message_t* msg,
                                    uint8_t type,
                                    uint8_t autopilot,
                                    uint8_t base_mode,
                                    uint32_t custom_mode,
                                    uint8_t system_status)

 * @brief Pack a vfr_hud message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param airspeed Current airspeed in m/s
 * @param groundspeed Current ground speed in m/s
 * @param heading Current heading in degrees, in compass units (0..360, 0=north)
 * @param throttle Current throttle setting in integer percent, 0 to 100
 * @param alt Current altitude (MSL), in meters
 * @param climb Current climb rate in meters/second
 * @return length of the message in bytes (excluding serial stream start sign)
 
uint16_t mavlink_msg_vfr_hud_pack(uint8_t system_id,
                                  uint8_t component_id,
                                  mavlink_message_t* msg,
						                      float airspeed,
                                  float groundspeed,
                                  int16_t heading, 
                                  uint16_t throttle,
                                  float alt, 
                                  float climb)


 * @brief Pack a gps_raw_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude in 1E7 degrees
 * @param lon Longitude in 1E7 degrees
 * @param alt Altitude in 1E3 meters (millimeters) above MSL
 * @param eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 * @param epv GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 * @param vel GPS ground speed (m/s * 100). If unknown, set to: 65535
 * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 * @return length of the message in bytes (excluding serial stream start sign)

uint16_t mavlink_msg_gps_raw_int_pack(uint8_t system_id,
                                      uint8_t component_id,
                                      mavlink_message_t* msg,
                                      uint64_t time_usec,
                                      uint8_t fix_type,
                                      int32_t lat,
                                      int32_t lon,
                                      int32_t alt,
                                      uint16_t eph,
                                      uint16_t epv,
                                      uint16_t vel, 
                                      uint16_t cog,
                                      uint8_t satellites_visible)

 * @brief Pack a rc_channels_scaled message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param port Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
 * @param chan1_scaled RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan2_scaled RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan3_scaled RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan4_scaled RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan5_scaled RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan6_scaled RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan7_scaled RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan8_scaled RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 * @return length of the message in bytes (excluding serial stream start sign)

(rcData[0]-1500) * 20
(rcData[1]-1500) * 20
(rcData[2]-1500) * 20
(rcData[3]-1500) * 20
(rcData[4]-1500) * 20
(rcData[5]-1500) * 20
(rcData[6]-1500) * 20
(rcData[7]-1500) * 20
rssi

uint16_t mavlink_msg_rc_channels_scaled_pack(uint8_t system_id,
                                             uint8_t component_id,
                                             mavlink_message_t* msg,
                                             uint32_t time_boot_ms,
                                             uint8_t port,
                                             int16_t chan1_scaled,
                                             int16_t chan2_scaled,
                                             int16_t chan3_scaled,
                                             int16_t chan4_scaled,
                                             int16_t chan5_scaled, 
                                             int16_t chan6_scaled, 
                                             int16_t chan7_scaled,
                                             int16_t chan8_scaled,
                                             uint8_t rssi)

 * @brief Pack a rc_channels_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param port Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
 * @param chan1_raw RC channel 1 value, in microseconds
 * @param chan2_raw RC channel 2 value, in microseconds
 * @param chan3_raw RC channel 3 value, in microseconds
 * @param chan4_raw RC channel 4 value, in microseconds
 * @param chan5_raw RC channel 5 value, in microseconds
 * @param chan6_raw RC channel 6 value, in microseconds
 * @param chan7_raw RC channel 7 value, in microseconds
 * @param chan8_raw RC channel 8 value, in microseconds
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 * @return length of the message in bytes (excluding serial stream start sign)
 
uint16_t mavlink_msg_rc_channels_raw_pack(uint8_t system_id,
                                          uint8_t component_id,
                                          mavlink_message_t* msg,
                                          uint32_t time_boot_ms,
                                          uint8_t port, 
                                          uint16_t chan1_raw, 
                                          uint16_t chan2_raw,
                                          uint16_t chan3_raw, 
                                          uint16_t chan4_raw,
                                          uint16_t chan5_raw,
                                          uint16_t chan6_raw,
                                          uint16_t chan7_raw,
                                          uint16_t chan8_raw,
                                          uint8_t rssi)

 * @brief Pack a attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)

uint16_t mavlink_msg_attitude_pack(uint8_t system_id,
                                   uint8_t component_id,
                                   mavlink_message_t* msg,
                                   uint32_t time_boot_ms,
                                   float roll,
                                   float pitch,
                                   float yaw,
                                   float rollspeed, 
                                   float pitchspeed,
                                   float yawspeed)

alle Variablen-Namen werden als text mit maximal 16Zeichen gesendet und empfangen

 * @brief Pack a param_value message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param param_id Onboard parameter id
 * @param param_value Onboard parameter value
 * @param param_type Onboard parameter type: see MAV_VAR enum
 * @param param_count Total number of onboard parameters
 * @param param_index Index of this onboard parameter
 * @return length of the message in bytes (excluding serial stream start sign)
 
uint16_t mavlink_msg_param_value_pack(uint8_t system_id,
                                      uint8_t component_id,
                                      mavlink_message_t* msg,
                                      const char *param_id,
                                      float param_value,
                                      uint8_t param_type,
                                      uint16_t param_count,
                                      uint16_t param_index)

typedef struct __mavlink_param_set_t
{
 float   param_value;      ///< Onboard parameter value
 uint8_t target_system;    ///< System ID
 uint8_t target_component; ///< Component ID
 char    param_id[16];     ///< Onboard parameter id
 uint8_t param_type;       ///< Onboard parameter type: see MAV_VAR enum
} mavlink_param_set_t;

enum MAV_VAR
{
	MAV_VAR_FLOAT    = 0, // *32 bit float | 
	MAV_VAR_UINT8    = 1, // * 8 bit unsigned integer | 
	MAV_VAR_INT8     = 2, // * 8 bit signed integer | 
	MAV_VAR_UINT16   = 3, // * 16 bit unsigned integer | 
	MAV_VAR_INT16    = 4, // * 16 bit signed integer | 
	MAV_VAR_UINT32   = 5, // * 32 bit unsigned integer | 
	MAV_VAR_INT32    = 6, // * 32 bit signed integer | 
	MAV_VAR_ENUM_END = 7, // *  |
};
*/
