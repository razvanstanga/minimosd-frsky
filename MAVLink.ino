#ifndef FRSKY
#include "../GCS_MAVLink/include/mavlink/v1.0/mavlink_types.h"
#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h"
#endif

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
#define CURRENT_ID         0x28
#define VOLTS_BP_ID        0x3A
#define VOLTS_AP_ID        0x3B
#define VFAS_ID            0x39
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
#define EARTH_RADIUS ((uint32_t)111194)

#define SPORT_DATA_U8(packet)   (buffer[4])
#define SPORT_DATA_S32(packet)  (*((int32_t *)(buffer+4)))
#define SPORT_DATA_U32(packet)  (*((uint32_t *)(buffer+4)))

int32_t  gpsAltitude;
int32_t  gpsAltitudeOffset;
uint8_t  gpsDistNeeded;
int8_t   gpsFix;           // -1=never fixed, 0=fix lost, 1=fixed
int16_t  gpsAltitude_bp;   // 0x01   before punct
uint32_t distFromEarthAxis;//        2 spares reused
int16_t  gpsAltitude_ap;   // 0x01+8 after punct
uint16_t gpsSpeed_bp;      // 0x11   before punct
uint16_t gpsLongitude_bp;  // 0x12   before punct
uint16_t gpsLatitude_bp;   // 0x13   before punct
uint16_t gpsCourse_bp;     // 0x14   before punct (0..359.99 deg. -- seemingly 2-decimal precision)
uint16_t gpsSpeed_ap;      // 0x11+8
uint16_t gpsLongitude_ap;  // 0x12+8
uint16_t gpsLatitude_ap;   // 0x13+8
uint16_t gpsCourse_ap;     // 0x14+8
uint32_t tmpLatitude;
uint32_t tmpLongitude;
uint32_t pilotLatitude;    //        2 spares reused
uint32_t pilotLongitude;   //        2 spares reused
uint16_t gpsLongitudeEW;   // 0x1A+8 East/West
uint16_t gpsLatitudeNS;    // 0x1B+8 North/South
uint16_t gpsDistance;
uint16_t volts_bp;         // 0x3A
uint16_t volts_ap;         // 0x3B
//uint16_t isqrt32(uint32_t n);

unsigned long _baro_altitude;
unsigned long _altitude_offset;
unsigned long _uptime;
unsigned int  _osd_swr;
float         _cell_voltage;
static float  _osd_analog_batt = 0;
bool          _sensorVario = false;
bool          _sensorCurrent = false;
bool          _sensorGps = false;
bool          _sensorFlvss = false;

uint8_t  cellsCount;
uint16_t cellVolts[12];
int16_t  cellsSum;
uint8_t  minCellIdx;
uint16_t minCellVolts;

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
                }
                if (appId == SWR_ID) {
                  _osd_swr = SPORT_DATA_U8(buffer);
                }
                if (appId == BATT_ID) {
                  _osd_analog_batt = SPORT_DATA_U8(buffer);
                }
                if (appId == CURRENT_ID) { // old vfas, works with data in from vario smart port
                  _sensorCurrent = true;
                  osd_curr_A = SPORT_DATA_U32(buffer);
                  osd_curr_A = osd_curr_A * 10;
                  unsigned long now = micros();
                  mah_used += (long) osd_curr_A * 10000 / (3600000000 /
                                                              (now - _uptime));
                  _uptime = now;
                }
                if (appId == VOLTS_AP_ID) { // old vfas, works with data in from vario smart port
                  _sensorCurrent = true;
                  volts_ap = SPORT_DATA_U32(buffer);
                  osd_vbat_A = (float) (((volts_bp * 100 + volts_ap * 10) * 21) / 110) / 10;
                }
                if (appId == VOLTS_BP_ID) { // old vfas, works with data in from vario smart port
                  _sensorCurrent = true;
                  volts_bp = SPORT_DATA_U32(buffer);
                }
    
                if (appId >= VARIO_FIRST_ID && appId <= VARIO_LAST_ID) {
                  _sensorVario = true;
                  osd_airspeed = SPORT_DATA_S32(buffer);
                } else if (appId >= ALT_FIRST_ID && appId <= ALT_LAST_ID) {
                  _sensorVario = true;
                  setBaroAltitude( SPORT_DATA_S32(buffer) );
                  osd_alt = float(_baro_altitude/100);
                } else if (appId >= VFAS_FIRST_ID && appId <= VFAS_LAST_ID) {
                  _sensorCurrent = true;
                  osd_vbat_A = SPORT_DATA_U32(buffer);
                  osd_vbat_A = (float) osd_vbat_A/100;
/*                  
                  if(osd_vbat_A > 21) cell_count = 6;
                  else if (osd_vbat_A > 16.8 && cell_count != 6) cell_count = 5;
                  else if(osd_vbat_A > 12.6 && cell_count != 5) cell_count = 4;
                  else if(osd_vbat_A > 8.4 && cell_count != 4) cell_count = 3;
                  else if(osd_vbat_A > 4.2 && cell_count != 3) cell_count = 2;
                  else cell_count = 0;
*/                  
                  //cell_count = cellsCount;
                  
                } else if (appId >= CURR_FIRST_ID && appId <= CURR_LAST_ID) {
                  _sensorCurrent = true;
                  osd_curr_A = SPORT_DATA_U32(buffer);
                  osd_curr_A = osd_curr_A * 10;
                  unsigned long now = micros();
                  mah_used += (long) osd_curr_A * 10000 / (3600000000 /
                                                              (now - _uptime));
                  _uptime = now;
                } else if (appId >= CELLS_FIRST_ID && appId <= CELLS_LAST_ID) {
                  _sensorFlvss = true;
                  
                  uint32_t cells = SPORT_DATA_U32(buffer);
                  uint8_t battnumber = cells & 0xF;
                  uint32_t minCell, minCellNum;

                  cellVolts[battnumber] = ((cells & 0x000FFF00) >> 8) / 10;
                  cellVolts[battnumber+1] = ((cells & 0xFFF00000) >> 20) / 10;
          
                  if (cellsCount < battnumber+2)
                    cellsCount = battnumber+2;
                  if (cellVolts[battnumber+1] == 0)
                    cellsCount--;
          
                  if ((cellVolts[battnumber] < cellVolts[battnumber+1]) || (cellVolts[battnumber+1] == 0)) {
                    minCell = cellVolts[battnumber];
                    minCellNum = battnumber;
                  } else {
                    minCell = cellVolts[battnumber+1];
                    minCellNum = battnumber+1;
                  }
          
                  if (!minCellVolts || minCell < minCellVolts || minCellNum==minCellIdx) {
                    minCellIdx = minCellNum;
                    minCellVolts = minCell;
                  }

                  _cell_voltage = ((((cells & 0x000FFF00) >> 8) / 10)*2);
                  _cell_voltage = float(_cell_voltage/100);
                  osd_battery_remaining_A = ((_cell_voltage-3.72)*100) / (4.2-3.72);
                
                  // use cells as temperature to use CT
                  temperature = _cell_voltage;
                  
                } else if (appId >= GPS_SPEED_FIRST_ID && appId <= GPS_SPEED_LAST_ID) {
                   _sensorGps =  true;                  
                   osd_groundspeed = SPORT_DATA_U32(buffer);
                   osd_groundspeed = (osd_groundspeed * 46) / 25 / 1000;
                } else if (appId >= GPS_COURS_FIRST_ID && appId <= GPS_COURS_LAST_ID) {
                    _sensorGps =  true;
                    uint32_t course = SPORT_DATA_U32(buffer);
                    gpsCourse_bp = course / 100;
                    gpsCourse_ap = course % 100;
                    osd_heading = gpsCourse_bp;
                } else if (appId >= GPS_TIME_DATE_FIRST_ID && appId <= GPS_TIME_DATE_LAST_ID) {
                    _sensorGps =  true;
                    uint32_t gps_time_date = SPORT_DATA_U32(buffer);
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
                  _sensorGps =  true;
                  gpsAltitude = SPORT_DATA_S32(buffer);
                  
                  if (!gpsAltitudeOffset)
                    gpsAltitudeOffset = gpsAltitude;

                  if (_sensorVario) {
                    osd_home_alt = osd_alt;
                  } else {
                    osd_home_alt = gpsAltitudeOffset/100;
                    osd_alt = gpsAltitude/100 - gpsAltitudeOffset/100;
                  }

/*        
                  if (!baroAltitudeOffset) {
                    int altitude = TELEMETRY_RELATIVE_GPS_ALT_BP;
                    if (altitude > frskyData.hub.maxAltitude)
                      frskyData.hub.maxAltitude = altitude;
                    if (altitude < frskyData.hub.minAltitude)
                      frskyData.hub.minAltitude = altitude;
                  }
*/                  
/*          
                  if (gpsFix > 0) {
                    if (!pilotLatitude && !pilotLongitude) {
                      // First received GPS position => Pilot GPS position
          	      getGpsPilotPosition();
                    }
                    getGpsDistance();
                  }
*/                   
                
                } else if (appId >= GPS_LONG_LATI_FIRST_ID && appId <= GPS_LONG_LATI_LAST_ID) {
                  _sensorGps =  true;
                   uint32_t gps_long_lati_data = SPORT_DATA_U32(buffer); 
                   uint32_t gps_long_lati_b1w, gps_long_lati_a1w;
                   
                   gps_long_lati_b1w = (gps_long_lati_data & 0x3fffffff) / 10000; 
                   gps_long_lati_a1w = (gps_long_lati_data & 0x3fffffff) % 10000; 
              
                   switch ((gps_long_lati_data & 0xc0000000) >> 30) { 
                     case 0: 
                       gpsLatitude_bp = (gps_long_lati_b1w / 60 * 100) + (gps_long_lati_b1w % 60);
                       gpsLatitude_ap = gps_long_lati_a1w;
                       
                       extractLatitudeLongitude(&tmpLatitude, &tmpLongitude);
                       osd_lat = (float) tmpLatitude/1000000;
                       gpsLatitudeNS = 'N';
                       break; 
                     case 1: 
                       gpsLatitude_bp = (gps_long_lati_b1w / 60 * 100) + (gps_long_lati_b1w % 60);
                       gpsLatitude_ap = gps_long_lati_a1w;
                      
                       extractLatitudeLongitude(&tmpLatitude, &tmpLongitude);
                       osd_lat = (float) tmpLatitude/1000000;
                       gpsLatitudeNS = 'S';
                       break; 
                     case 2: 
                       gpsLongitude_bp = (gps_long_lati_b1w / 60 * 100) + (gps_long_lati_b1w % 60);
                       gpsLongitude_ap = gps_long_lati_a1w;
                       
                       extractLatitudeLongitude(&tmpLatitude, &tmpLongitude);
                       osd_lon = (float) tmpLongitude/1000000;
                       gpsLongitudeEW = 'E';
                       break; 
                     case 3: 
                       gpsLongitude_bp = (gps_long_lati_b1w / 60 * 100) + (gps_long_lati_b1w % 60);
                       gpsLongitude_ap = gps_long_lati_a1w;
                       
                       extractLatitudeLongitude(&tmpLatitude, &tmpLongitude);
                       osd_lon = (float) tmpLongitude/1000000;
                       gpsLongitudeEW = 'W';
                       break; 
                   }
                  if (gpsLongitudeEW && gpsLatitudeNS) {
                    osd_fix_type = 3;
                    gpsFix = 1;
                    /*
                    if (!pilotLatitude && !pilotLongitude) {
                      // First received GPS position => Pilot GPS position
                      getGpsPilotPosition();
                    }
                    */                   
                  }
                  else if (gpsFix > 0) {
                    osd_fix_type = 0;
                    gpsFix = 0;
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
void setBaroAltitude(float baro_altitude)
{
  if (!_altitude_offset)
    _altitude_offset = -baro_altitude;

  baro_altitude += _altitude_offset;
  _baro_altitude = baro_altitude;
  if (_baro_altitude < 0 || _baro_altitude > 1000000) {
    _baro_altitude = 0;
  }
}

void extractLatitudeLongitude(uint32_t * latitude, uint32_t * longitude)
{
  div_t qr = div(gpsLatitude_bp, 100);
  *latitude = ((uint32_t)(qr.quot) * 1000000) + (((uint32_t)(qr.rem) * 10000 + gpsLatitude_ap) * 5) / 3;

  qr = div(gpsLongitude_bp, 100);
  *longitude = ((uint32_t)(qr.quot) * 1000000) + (((uint32_t)(qr.rem) * 10000 + gpsLongitude_ap) * 5) / 3;
}

void getGpsPilotPosition()
{
  extractLatitudeLongitude(&pilotLatitude, &pilotLongitude);
  uint32_t lat = pilotLatitude / 10000;
  uint32_t angle2 = (lat*lat) / 10000;
  uint32_t angle4 = angle2 * angle2;
  distFromEarthAxis = 139*(((uint32_t)10000000-((angle2*(uint32_t)123370)/81)+(angle4/25))/12500);
}

void getGpsDistance()
{
  uint32_t lat, lng;

  extractLatitudeLongitude(&lat, &lng);

  uint32_t angle = (lat > pilotLatitude) ? lat - pilotLatitude : pilotLatitude - lat;
  uint32_t dist = EARTH_RADIUS * angle / 1000000;
  uint32_t result = dist*dist;

  angle = (lng > pilotLongitude) ? lng - pilotLongitude : pilotLongitude - lng;
  dist = distFromEarthAxis * angle / 1000000;
  result += dist*dist;

  dist = abs(gpsAltitude_bp);
  result += dist*dist;

  gpsDistance = isqrt32(result);
}

uint16_t isqrt32(uint32_t n)
{
    uint16_t c = 0x8000;
    uint16_t g = 0x8000;

    for(;;) {
        if((uint32_t)g*g > n)
            g ^= c;
        c >>= 1;
        if(c == 0)
            return g;
        g |= c;
    }
}

#ifdef FRSKYNODEF
void displayGpsTime()
{
  uint8_t att = (TELEMETRY_STREAMING() ? LEFT|LEADING0 : LEFT|LEADING0|BLINK);
  lcd_outdezNAtt(CENTER_OFS+6*FW+5, STATUS_BAR_Y, frskyData.hub.hour, att, 2);
  lcd_putcAtt(CENTER_OFS+8*FW+2, STATUS_BAR_Y, ':', att);
  lcd_outdezNAtt(CENTER_OFS+9*FW+2, STATUS_BAR_Y, frskyData.hub.min, att, 2);
  lcd_putcAtt(CENTER_OFS+11*FW-1, STATUS_BAR_Y, ':', att);
  lcd_outdezNAtt(CENTER_OFS+12*FW-1, STATUS_BAR_Y, frskyData.hub.sec, att, 2);
  lcd_status_line();
}

/*
  // Latitude
  displayGpsCoord(line, frskyData.hub.gpsLatitudeNS, frskyData.hub.gpsLatitude_bp, frskyData.hub.gpsLatitude_ap);
  // Longitude
  displayGpsCoord(line, frskyData.hub.gpsLongitudeEW, frskyData.hub.gpsLongitude_bp, frskyData.hub.gpsLongitude_ap);
*/

void displayGpsCoord(uint8_t y, char direction, int16_t bp, int16_t ap)
{
  if (frskyData.hub.gpsFix >= 0) {
    if (!direction) direction = '-';
    lcd_outdezAtt(TELEM_2ND_COLUMN, y, bp / 100, LEFT); // ddd before '.'
    lcd_putc(lcdLastPos, y, '@');
    uint8_t mn = bp % 100;
    if (g_eeGeneral.gpsFormat == 0) {
      lcd_putc(lcdLastPos+FWNUM, y, direction);
      lcd_outdezNAtt(lcdLastPos+FW+FW+1, y, mn, LEFT|LEADING0, 2); // mm before '.'
      lcd_vline(lcdLastPos, y, 2);
      uint16_t ss = ap * 6;
      lcd_outdezAtt(lcdLastPos+3, y, ss / 1000, LEFT); // ''
      lcd_plot(lcdLastPos, y+FH-2, 0); // small decimal point
      lcd_outdezAtt(lcdLastPos+2, y, ss % 1000, LEFT); // ''
      lcd_vline(lcdLastPos, y, 2);
      lcd_vline(lcdLastPos+2, y, 2);
    }
    else {
      lcd_outdezNAtt(lcdLastPos+FW, y, mn, LEFT|LEADING0, 2); // mm before '.'
      lcd_plot(lcdLastPos, y+FH-2, 0); // small decimal point
      lcd_outdezNAtt(lcdLastPos+2, y, ap, LEFT|UNSIGN|LEADING0, 4); // after '.'
      lcd_putc(lcdLastPos+1, y, direction);
    }
  }
  else {
    // no fix
    lcd_puts(TELEM_2ND_COLUMN, y, STR_VCSWFUNC+1/*----*/);
  }
}
#endif

#endif
