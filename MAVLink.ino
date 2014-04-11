#include "../GCS_MAVLink/include/mavlink/v1.0/mavlink_types.h"
#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h"

#ifdef FRSKY

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
#define CELLS_FIRST_ID     0x0300
#define CELLS_LAST_ID      0x030f

#define GPS_LONG_LATI_FIRST_ID  0x0800
#define GPS_LONG_LATI_LAST_ID   0x080f
#define GPS_ALT_FIRST_ID        0x0820
#define GPS_ALT_LAST_ID         0x082f
#define GPS_SPEED_FIRST_ID      0x0830
#define GPS_SPEED_LAST_ID       0x083f
#define GPS_COURS_FIRST_ID      0x0840
#define GPS_COURS_LAST_ID       0x084f
#define GPS_TIME_DATE_FIRST_ID  0x0850
#define GPS_TIME_DATE_LAST_ID   0x085f

#define SPORT_DATA_U8(packet)   (buffer[4])
#define SPORT_DATA_S32(packet)  (*((int32_t *)(buffer+4)))
#define SPORT_DATA_U32(packet)  (*((uint32_t *)(buffer+4)))

long          _varioSpeed;
long          _varioAltitude;
unsigned long _altitudeOffset;
unsigned long _baroAltitude;
int           _vfasVoltage;
int           _vfasCurrent;
unsigned long _vfasConsumption;
unsigned long _uptime;
unsigned int   osd_swr;
//unsigned int   osd_rssi;
float          _cellVoltage;
static float   osd_analog_batt = 0;                 // Battery A voltage in milivolt

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

    static byte buffer[FRSKY_RX_PACKET_SIZE];
    static byte bufferIndex = 0;
    static byte dataState = STATE_DATA_IDLE;

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
        }
        
        if (data == START_STOP) {
          lastMAVBeat = millis();
          mavlink_active = 1;
          dataState = STATE_DATA_IN_FRAME;
          bufferIndex = 0;
          
                    mavbeat = 1;
                    apm_mav_system    = 0;
                    apm_mav_component = 0;
//                    apm_mav_type      = mavlink_msg_heartbeat_get_type(&msg);            
                 //   osd_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
                    //osd_mode = (uint8_t)mavlink_msg_heartbeat_get_custom_mode(&msg);
                    //Mode (arducoper armed/disarmed)
                    //base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
//                    if(getBit(base_mode,7)) motor_armed = 1;
//                    else motor_armed = 0;

                    osd_nav_mode = 0;          
                    /*lastMAVBeat = millis();
                    if(waitingMAVBeats == 1){
                        enable_mav_request = 1;
                    }*/
          
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
    
                if (appId == RSSI_ID) {
                  osd_rssi = SPORT_DATA_U8(buffer);
                  //Serial.println ("osd_rssi");
                  //Serial.println (osd_rssi);
                }
                if (appId == SWR_ID) {
                  osd_swr = SPORT_DATA_U8(buffer);
                  //Serial.println ("osd_swr");
                  //Serial.println (osd_swr);                  
                }
                if (appId == BATT_ID) {
                  osd_analog_batt = SPORT_DATA_U8(buffer);
                  //Serial.println ("osd_analog_batt");
                  //Serial.println (osd_analog_batt);                                    
                }
    
                if (appId >= VARIO_FIRST_ID && appId <= VARIO_LAST_ID) {
                  osd_airspeed = SPORT_DATA_S32(buffer);
                  //Serial.println ("osd_airspeed");
                  //Serial.println (osd_airspeed);
                  
                } else if (appId >= ALT_FIRST_ID && appId <= ALT_LAST_ID) {
                  //if (_altitudeOffset == 0) _altitudeOffset = -SPORT_DATA_S32(buffer);
                  
                  setBaroAltitude( SPORT_DATA_S32(buffer) );
                  //Serial.println ("osd_alt");
                  //Serial.println (_baroAltitude/100, 2);
                  osd_alt = float(_baroAltitude/100);
                } else if (appId >= VFAS_FIRST_ID && appId <= VFAS_LAST_ID) {
                  osd_vbat_A = SPORT_DATA_U32(buffer);
                  //Serial.println ("osd_vbat_A");
                  //Serial.println (float(osd_vbat_A/100));
                  osd_vbat_A = float(osd_vbat_A/100);
                  
                } else if (appId >= CURR_FIRST_ID && appId <= CURR_LAST_ID) {
                  osd_curr_A = SPORT_DATA_U32(buffer);
                  osd_curr_A = float(osd_curr_A/100);
                  unsigned long now = micros();
                  //osd_battery_remaining_A += (long)osd_curr_A * 1000 / (3600000000 / (now - _uptime));
                  //Serial.println ("osd_curr_A");
                  //Serial.println (osd_curr_A);
                  
                  _uptime = now;
                } else if (appId >= CELLS_FIRST_ID && appId <= CELLS_LAST_ID) {
                  uint32_t cells = SPORT_DATA_U32(buffer);
                  uint8_t battnumber = cells & 0xF;
                  //Serial.println ("battnumber");
                  //Serial.println (battnumber);
                  _cellVoltage = ((((cells & 0x000FFF00) >> 8) / 10)*2);
                  //Serial.println ("cellVoltage");
                  //Serial.println ( cellVoltage/100, 2 );
                   osd_battery_remaining_A = float(_cellVoltage/100);
                  
                } else if (appId >= GPS_SPEED_FIRST_ID && appId <= GPS_SPEED_LAST_ID) {
                   osd_groundspeed = SPORT_DATA_U32(buffer);
                   osd_groundspeed = (osd_groundspeed * 46) / 25 / 1000;
                   //Serial.println ("osd_groundspeed");
                   //Serial.println (osd_groundspeed);                   
                   
                } else if (appId >= GPS_COURS_FIRST_ID && appId <= GPS_COURS_LAST_ID) {
                   osd_heading = SPORT_DATA_U32(buffer);
                   //frskyData.hub.gpsCourse_bp = course / 100;
                   //frskyData.hub.gpsCourse_ap = course % 100;  
                   //Serial.println ("osd_heading");
                   //Serial.println (osd_heading);                                      
                   
                } else if (appId >= GPS_TIME_DATE_FIRST_ID && appId <= GPS_TIME_DATE_LAST_ID) {
                    uint32_t gps_time_date = SPORT_DATA_U32(buffer);
                    //Serial.println ("gps_time_date");
                    //Serial.println (gps_time_date);                                                          
                    
                    if (gps_time_date & 0x000000ff) {
                      //frskyData.hub.year = (uint16_t) ((gps_time_date & 0xff000000) >> 24);
                      //frskyData.hub.month = (uint8_t) ((gps_time_date & 0x00ff0000) >> 16);
                      //frskyData.hub.day = (uint8_t) ((gps_time_date & 0x0000ff00) >> 8);
                    }
                    else {
                      //frskyData.hub.hour = (uint8_t) ((gps_time_date & 0xff000000) >> 24);
                      //frskyData.hub.min = (uint8_t) ((gps_time_date & 0x00ff0000) >> 16);
                      //frskyData.hub.sec = (uint16_t) ((gps_time_date & 0x0000ff00) >> 8);
                      //frskyData.hub.hour = ((uint8_t) (frskyData.hub.hour + g_eeGeneral.timezone + 24)) % 24;
                    }
                   
                } else if (appId >= GPS_ALT_FIRST_ID && appId <= GPS_ALT_LAST_ID) {
                   //osd_alt = SPORT_DATA_S32(buffer);   
                   
                } else if (appId >= GPS_LONG_LATI_FIRST_ID && appId <= GPS_LONG_LATI_LAST_ID) {
                    uint32_t gps_long_lati_data = SPORT_DATA_U32(buffer);
                    uint32_t gps_long_lati_b1w, gps_long_lati_a1w;
                    gps_long_lati_b1w = (gps_long_lati_data & 0x3fffffff) / 10000;
                    gps_long_lati_a1w = (gps_long_lati_data & 0x3fffffff) % 10000;
                    osd_heading = 0;
                    switch ((gps_long_lati_data & 0xc0000000) >> 30) {
                      case 0:
                        osd_lat = (gps_long_lati_b1w / 60 * 100) + (gps_long_lati_b1w % 60);
                        //osd_lat = gps_long_lati_a1w;
                        //frskyData.hub.gpsLatitudeNS = 'N';
                        osd_heading = 0;
                        break;
                      case 1:
                        osd_lat = (gps_long_lati_b1w / 60 * 100) + (gps_long_lati_b1w % 60);
                        //osd_lat = gps_long_lati_a1w;
                        //frskyData.hub.gpsLatitudeNS = 'S';
                        osd_heading = 90;
                        break;
                      case 2:
                        osd_lon = (gps_long_lati_b1w / 60 * 100) + (gps_long_lati_b1w % 60);
                        //osd_lon = gps_long_lati_a1w;
                        //frskyData.hub.gpsLongitudeEW = 'E';
                        osd_heading = 180;
                        break;
                      case 3:
                        osd_lon = (gps_long_lati_b1w / 60 * 100) + (gps_long_lati_b1w % 60);
                        //osd_lon = gps_long_lati_a1w;
                        //frskyData.hub.gpsLongitudeEW = 'W';
                        osd_heading = 360;
                        break;
                    }                   
                }
                break;
            }
            
            
          }
        }

#else
        uint8_t c = Serial.read();

        /* allow CLI to be started by hitting enter 3 times, if no
        heartbeat packets have been received */
        if (mavlink_active == 0 && millis() < 20000 && millis() > 5000) {
            if (c == '\n' || c == '\r') {
                crlf_count++;
            } else {
                crlf_count = 0;
            }
//            if (crlf_count == 3) {
//                uploadFont();
//            }
        }

        //trying to grab msg  
        if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            lastMAVBeat = millis();
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
//                    if(getBit(base_mode,7)) motor_armed = 1;
//                    else motor_armed = 0;

                    osd_nav_mode = 0;          
                    /*lastMAVBeat = millis();
                    if(waitingMAVBeats == 1){
                        enable_mav_request = 1;
                    }*/
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
                    osd_cog = mavlink_msg_gps_raw_int_get_cog(&msg);
                    eph = mavlink_msg_gps_raw_int_get_eph(&msg);
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
                  alt_error = mavlink_msg_nav_controller_output_get_alt_error(&msg);
                  aspd_error = mavlink_msg_nav_controller_output_get_aspd_error(&msg);
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
//                  if (osd_climb < 1.66 && osd_climb > -1.66){
                  osd_winddirection = mavlink_msg_wind_get_direction(&msg); // 0..360 deg, 0=north
                  osd_windspeed = mavlink_msg_wind_get_speed(&msg); //m/s
//                  osd_windspeedz = mavlink_msg_wind_get_speed_z(&msg); //m/s
//                  }
                }
                break;
            case MAVLINK_MSG_ID_SCALED_PRESSURE:
                {
                    temperature = mavlink_msg_scaled_pressure_get_temperature(&msg);
                }
                break;
            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                {
                    osd_home_alt = osd_alt - (mavlink_msg_global_position_int_get_relative_alt(&msg)*0.001);
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

#ifdef FRSKY
void setBaroAltitude(float baroAltitude)
{
  if (!_altitudeOffset)
    _altitudeOffset = -baroAltitude;

  baroAltitude += _altitudeOffset;
  _baroAltitude = baroAltitude;
  if (_baroAltitude < 0 || _baroAltitude > 1000000) {
    _baroAltitude = 0;
  }
}
#endif
