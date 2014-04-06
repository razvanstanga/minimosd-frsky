#include "../GCS_MAVLink/include/mavlink/v1.0/mavlink_types.h"
#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h"

#ifdef FRSKY
#ifndef SPort_H
#define SPort_H

#define FRSKY_RX_PACKET_SIZE 0x09

#define STATE_DATA_IDLE      0x01
#define STATE_DATA_IN_FRAME  0x02
#define STATE_DATA_XOR       0x03

#define START_STOP           0x7e
#define BYTESTUFF            0x7d
#define STUFF_MASK           0x20
#define DATA_FRAME           0x10

// FrSky new DATA IDs (2 bytes)
#define RSSI_ID            0xf101
#define ADC1_ID            0xf102
#define ADC2_ID            0xf103
#define BATT_ID            0xf104
#define SWR_ID             0xf105
#define T1_FIRST_ID        0x0400
#define T1_LAST_ID         0x040f
#define T2_FIRST_ID        0x0410
#define T2_LAST_ID         0x041f
#define RPM_FIRST_ID       0x0500
#define RPM_LAST_ID        0x050f
#define FUEL_FIRST_ID      0x0600
#define FUEL_LAST_ID       0x060f
#define ALT_FIRST_ID       0x0100
#define ALT_LAST_ID        0x010f
#define VARIO_FIRST_ID     0x0110
#define VARIO_LAST_ID      0x011f
#define ACCX_FIRST_ID      0x0700
#define ACCX_LAST_ID       0x070f
#define ACCY_FIRST_ID      0x0710
#define ACCY_LAST_ID       0x071f
#define ACCZ_FIRST_ID      0x0720
#define ACCZ_LAST_ID       0x072f
#define CURR_FIRST_ID      0x0200
#define CURR_LAST_ID       0x020f
#define VFAS_FIRST_ID      0x0210
#define VFAS_LAST_ID       0x021f
#define GPS_SPEED_FIRST_ID 0x0830
#define GPS_SPEED_LAST_ID  0x083f
#define CELLS_FIRST_ID     0x0300
#define CELLS_LAST_ID      0x030f

#define SPORT_DATA_U8(packet)   (buffer[4])
#define SPORT_DATA_S32(packet)  (*((int32_t *)(buffer+4)))
#define SPORT_DATA_U32(packet)  (*((uint32_t *)(buffer+4)))

long          _varioSpeed;
long          _varioAltitude;
unsigned long _altitudeOffset;
int           _vfasVoltage;
int           _vfasCurrent;
unsigned long _vfasConsumption;
unsigned long _uptime;
int           mySerialStarted;

static byte buffer[FRSKY_RX_PACKET_SIZE];
static byte bufferIndex = 0;
static byte dataState = STATE_DATA_IDLE;

#endif
#endif

// true when we have received at least 1 MAVLink packet
static bool mavlink_active;
static uint8_t crlf_count = 0;

static int packet_drops = 0;
static int parse_error = 0;

void request_mavlink_rates()
{
    const int  maxStreams = 6;
    const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_RAW_SENSORS,
        MAV_DATA_STREAM_EXTENDED_STATUS,
        MAV_DATA_STREAM_RC_CHANNELS,
        MAV_DATA_STREAM_POSITION,
        MAV_DATA_STREAM_EXTRA1, 
        MAV_DATA_STREAM_EXTRA2};
    const uint16_t MAVRates[maxStreams] = {0x02, 0x02, 0x05, 0x02, 0x05, 0x02};
    for (int i=0; i < maxStreams; i++) {
        mavlink_msg_request_data_stream_send(MAVLINK_COMM_0,
            apm_mav_system, apm_mav_component,
            MAVStreams[i], MAVRates[i], 1);
    }
}

void read_mavlink(){
    mavlink_message_t msg; 
    mavlink_status_t status;

#ifdef FRSKY_DEBUG
    if ( Serial.available() ) {
      osd.setPanel(5,9);
      osd.openPanel();
      osd.printf("%s", "serial available");
      osd.closePanel();
    } else {
      osd.setPanel(5,9);
      osd.openPanel();
      osd.printf("%s", "serial not available");
      osd.closePanel();
    }
#endif

    //grabing data 
    while(Serial.available() > 0) {
#ifdef FRSKY
        byte data = Serial.read();
    
        /* allow CLI to be started by hitting enter 3 times, if no
        heartbeat packets have been received */
        if (mavlink_active == 0 && millis() < 20000 && millis() > 5000) {
            if (data == '\n' || data == '\r') {
                crlf_count++;
            } else {
                crlf_count = 0;
            }
            if (crlf_count == 3) {
                uploadFont();
            }
        } else {
            //mavlink_active = 1;
        }
    
        if (data == START_STOP) {
          dataState = STATE_DATA_IN_FRAME;
          bufferIndex = 0;
          
          mavbeat = 1;
          //apm_mav_system    = msg.sysid;
          //apm_mav_component = msg.compid;
          osd_mode = 1;
          //Mode (arducoper armed/disarmed)
          //base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
          motor_armed = 1;
    
          osd_nav_mode = 0;
          lastMAVBeat = millis();
          if(waitingMAVBeats == 1){
              enable_mav_request = 1;
          }
          
        } else {
          switch (dataState) {
            case STATE_DATA_XOR:
              buffer[bufferIndex++] = data ^ STUFF_MASK;
              dataState = STATE_DATA_IN_FRAME;
              break;
            case STATE_DATA_IN_FRAME:
              if (data == BYTESTUFF)
                dataState = STATE_DATA_XOR;
              else
                buffer[bufferIndex++] = data;
              break;
          }
        }
    
        if (bufferIndex == FRSKY_RX_PACKET_SIZE) {
          mavlink_active = 1;
          dataState = STATE_DATA_IDLE;
    
          short crc = 0;
          for (int i = 1; i < FRSKY_RX_PACKET_SIZE; i++) {
            crc += buffer[i];
            crc += crc >> 8;
            crc &= 0x00ff;
            crc += crc >> 8;
            crc &= 0x00ff;
          }
          if (crc == 0x00ff) {
            byte packetType = buffer[1];
            switch (packetType) {
              case DATA_FRAME:
                unsigned int appId = *((unsigned int *)(buffer+2));
    
                if (appId >= VARIO_FIRST_ID && appId <= VARIO_LAST_ID) {
                  osd_groundspeed = SPORT_DATA_S32(buffer);
    
                } else if (appId >= ALT_FIRST_ID && appId <= ALT_LAST_ID) {
                    if (_altitudeOffset == 0)
                      _altitudeOffset = -SPORT_DATA_S32(buffer);
    
                  osd_alt = SPORT_DATA_S32(buffer) + _altitudeOffset;
    
                } else if (appId >= VFAS_FIRST_ID && appId <= VFAS_LAST_ID) {
                  osd_vbat_A = SPORT_DATA_S32(buffer) * 10;
    
                } else if (appId >= CURR_FIRST_ID && appId <= CURR_LAST_ID) {
                  osd_curr_A = SPORT_DATA_U32(buffer);
    
                  unsigned long now = micros();
                  osd_battery_remaining_A += (long)osd_curr_A * 1000 / (3600000000 /
                                                                  (now - _uptime));
                  _uptime = now;
    
                } else if (appId == RSSI_ID) {
                  osd_rssi = SPORT_DATA_U32(buffer);
                }
                break;
            }
          }
        }
#endif

#ifndef FRSKY
        uint8_t c = Serial.read();
        
        /* allow CLI to be started by hitting enter 3 times, if no
        heartbeat packets have been received */
        if (mavlink_active == 0 && millis() < 20000 && millis() > 5000) {
            if (c == '\n' || c == '\r') {
                crlf_count++;
            } else {
                crlf_count = 0;
            }
            if (crlf_count == 3) {
                uploadFont();
            }
        }        
        
        //trying to grab msg  
        if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            mavlink_active = 1;
            //handle msg
            switch(msg.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT:
                {
                    mavbeat = 1;
                    apm_mav_system    = msg.sysid;
                    apm_mav_component = msg.compid;
//                    apm_mav_type      = mavlink_msg_heartbeat_get_type(&msg);            
                 //   osd_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
                    osd_mode = (uint8_t)mavlink_msg_heartbeat_get_custom_mode(&msg);
                    //Mode (arducoper armed/disarmed)
                    base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
                    if(getBit(base_mode,7)) motor_armed = 1;
                    else motor_armed = 0;

                    osd_nav_mode = 0;          
                    lastMAVBeat = millis();
                    if(waitingMAVBeats == 1){
                        enable_mav_request = 1;
                    }
                }
                break;
            case MAVLINK_MSG_ID_SYS_STATUS:
                {

                    osd_vbat_A = (mavlink_msg_sys_status_get_voltage_battery(&msg) / 1000.0f); //Battery voltage, in millivolts (1 = 1 millivolt)
                    osd_curr_A = mavlink_msg_sys_status_get_current_battery(&msg); //Battery current, in 10*milliamperes (1 = 10 milliampere)         
                    osd_battery_remaining_A = mavlink_msg_sys_status_get_battery_remaining(&msg); //Remaining battery energy: (0%: 0, 100%: 100)
                    //osd_mode = apm_mav_component;//Debug
                    //osd_nav_mode = apm_mav_system;//Debug
                }
                break;

            case MAVLINK_MSG_ID_GPS_RAW_INT:
                {
                    osd_lat = mavlink_msg_gps_raw_int_get_lat(&msg) / 10000000.0f;
                    osd_lon = mavlink_msg_gps_raw_int_get_lon(&msg) / 10000000.0f;
                    osd_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
                    osd_satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
                }
                break; 
            case MAVLINK_MSG_ID_VFR_HUD:
                {
                    osd_airspeed = mavlink_msg_vfr_hud_get_airspeed(&msg);
                    osd_groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);
                    osd_heading = mavlink_msg_vfr_hud_get_heading(&msg); // 0..360 deg, 0=north
                    osd_throttle = (uint8_t)mavlink_msg_vfr_hud_get_throttle(&msg);
                    osd_alt = mavlink_msg_vfr_hud_get_alt(&msg);
                    osd_climb = mavlink_msg_vfr_hud_get_climb(&msg);
                }
                break;
            case MAVLINK_MSG_ID_ATTITUDE:
                {
                    osd_pitch = ToDeg(mavlink_msg_attitude_get_pitch(&msg));
                    osd_roll = ToDeg(mavlink_msg_attitude_get_roll(&msg));
//                    osd_yaw = ToDeg(mavlink_msg_attitude_get_yaw(&msg));
                }
                break;
            case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
                {
//                  nav_roll = mavlink_msg_nav_controller_output_get_nav_roll(&msg);
//                  nav_pitch = mavlink_msg_nav_controller_output_get_nav_pitch(&msg);
//                  nav_bearing = mavlink_msg_nav_controller_output_get_nav_bearing(&msg);
                  wp_target_bearing = mavlink_msg_nav_controller_output_get_target_bearing(&msg);
                  wp_dist = mavlink_msg_nav_controller_output_get_wp_dist(&msg);
//                  alt_error = mavlink_msg_nav_controller_output_get_alt_error(&msg);
//                  aspd_error = mavlink_msg_nav_controller_output_get_aspd_error(&msg);
                  xtrack_error = mavlink_msg_nav_controller_output_get_xtrack_error(&msg);
                }
                break;
            case MAVLINK_MSG_ID_MISSION_CURRENT:
                {
                    wp_number = (uint8_t)mavlink_msg_mission_current_get_seq(&msg);
                }
                break;
            case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
                {
//                    chan1_raw = mavlink_msg_rc_channels_raw_get_chan1_raw(&msg);
//                    chan2_raw = mavlink_msg_rc_channels_raw_get_chan2_raw(&msg);
//                    chan3_raw = mavlink_msg_rc_channels_raw_get_chan3_raw(&msg);
//                    chan4_raw = mavlink_msg_rc_channels_raw_get_chan4_raw(&msg);
                    chan5_raw = mavlink_msg_rc_channels_raw_get_chan5_raw(&msg);
                    chan6_raw = mavlink_msg_rc_channels_raw_get_chan6_raw(&msg);
                    chan7_raw = mavlink_msg_rc_channels_raw_get_chan7_raw(&msg);
                    chan8_raw = mavlink_msg_rc_channels_raw_get_chan8_raw(&msg);
                    osd_rssi = mavlink_msg_rc_channels_raw_get_rssi(&msg);
                }
                break;           
            case MAVLINK_MSG_ID_WIND:
                {
                    osd_winddirection = abs(mavlink_msg_wind_get_direction(&msg)); // 0..360 deg, 0=north
                    osd_windspeed = mavlink_msg_wind_get_speed(&msg); //m/s
//                    osd_windspeedz = mavlink_msg_wind_get_speed_z(&msg); //m/s
                }
                break;
            case MAVLINK_MSG_ID_SCALED_PRESSURE:
                {
                    temperature = mavlink_msg_scaled_pressure_get_temperature(&msg);
                }
                break;
            default:
                //Do nothing
                break;
            }
        }
#endif
        delayMicroseconds(138);
        //next one
    }
    // Update global packet drops counter
    packet_drops += status.packet_rx_drop_count;
    parse_error += status.parse_error;

}
