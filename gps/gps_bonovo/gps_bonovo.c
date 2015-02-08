/*
 * Copyright (C) 2010 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* this implements a GPS hardware library for the Android emulator.
 * the following code should be built as a shared library that will be
 * placed into /system/lib/hw/gps.goldfish.so
 *
 * it will be loaded by the code in hardware/libhardware/hardware.c
 * which is itself called from android_location_GpsLocationProvider.cpp
 */


#include <errno.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>      // IOCTL
#include <termios.h>        // UART
#include <math.h>
#include <time.h>

#define  LOG_TAG  "gps_bonovo_libgps"
#include <cutils/sockets.h>
#include "gps_bonovo.h"
#include <cutils/properties.h>

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       N M E A   T O K E N I Z E R                     *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

typedef struct {
    const char*  p;
    const char*  end;
} Token;

#define  MAX_NMEA_TOKENS  32
typedef struct {
    int     count;
    Token   tokens[ MAX_NMEA_TOKENS ];
} NmeaTokenizer;

static int
nmea_tokenizer_init( NmeaTokenizer*  t, const char*  p, const char*  end )
{
    int    count = 0;
    char*  q;

    if(GPS_DEBUG) LOGI("%s:", __FUNCTION__);
    // the initial '$' is optional
    if (p < end && p[0] == '$')
        p += 1;

    // remove trailing newline
    if (end > p && end[-1] == '\n') {
        end -= 1;
        if (end > p && end[-1] == '\r')
            end -= 1;
    }

    // get rid of checksum at the end of the sentecne
    if (end >= p+3 && end[-3] == '*') {
        end -= 3;
    }

    while (p < end) {
        const char*  q = p;

        q = memchr(p, ',', end-p);
        if (q == NULL)
            q = end;

        if (q >= p) {    //add empty string
            if (count < MAX_NMEA_TOKENS) {
                t->tokens[count].p   = p;
                t->tokens[count].end = q;
                count += 1;
            }
        }
        if (q < end)
            q += 1;

        p = q;
    }

    t->count = count;
    return count;
}

static Token
nmea_tokenizer_get( NmeaTokenizer*  t, int  index )
{
    Token  tok;
    static const char*  dummy = "";

    if(GPS_DEBUG) LOGI("%s:", __FUNCTION__);
    if (index < 0 || index >= t->count) {
        tok.p = tok.end = dummy;
    } else
        tok = t->tokens[index];

    return tok;
}


static int
str2int( const char*  p, const char*  end )
{
    int   result = 0;
    int   len    = end - p;

    if(GPS_DEBUG) LOGI("%s:", __FUNCTION__);
    for ( ; len > 0; len--, p++ )
    {
        int  c;

        if (p >= end)
            goto Fail;

        c = *p - '0';
        if ((unsigned)c >= 10)
            goto Fail;

        result = result*10 + c;
    }
    return  result;

Fail:
    return -1;
}

static double
str2float( const char*  p, const char*  end )
{
    int   result = 0;
    int   len    = end - p;
    char  temp[16];

    if(GPS_DEBUG) LOGI("%s:", __FUNCTION__);
    if (len >= (int)sizeof(temp))
        return 0.;

    memcpy( temp, p, len );
    temp[len] = 0;
    return strtod( temp, NULL );
}

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       N M E A   P A R S E R                           *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

#define  NMEA_MAX_SIZE  256	//83

typedef struct {
    int     pos;
    int     overflow;
    int     utc_year;
    int     utc_mon;
    int     utc_day;
    int     utc_diff;
    GpsLocation  fix;
#if GPS_SV_INCLUDE
    GpsSvStatus  sv_status; 
    int     sv_status_changed;
#endif
    gps_location_callback  callback;
#if GPS_SV_INCLUDE
    gps_sv_status_callback sv_callback;
#endif
    //char    in[ NMEA_MAX_SIZE+1 ];
	char	in[NMEA_MAX_SIZE];
} NmeaReader;


static void
nmea_reader_update_utc_diff( NmeaReader*  r )
{
    time_t         now = time(NULL);
    struct tm      tm_local;
    struct tm      tm_utc;
    long           time_local, time_utc;

    if(GPS_DEBUG) LOGI("%s:", __FUNCTION__);
    gmtime_r( &now, &tm_utc );
    localtime_r( &now, &tm_local );

    time_local = tm_local.tm_sec +
                 60*(tm_local.tm_min +
                 60*(tm_local.tm_hour +
                 24*(tm_local.tm_yday +
                 365*tm_local.tm_year)));

    time_utc = tm_utc.tm_sec +
               60*(tm_utc.tm_min +
               60*(tm_utc.tm_hour +
               24*(tm_utc.tm_yday +
               365*tm_utc.tm_year)));

    r->utc_diff = time_utc - time_local;
}


static void
nmea_reader_init( NmeaReader*  r )
{
    if(GPS_DEBUG) LOGI("%s:", __FUNCTION__);
    memset( r, 0, sizeof(*r) );

    r->pos      = 0;
    r->overflow = 0;
    r->utc_year = -1;
    r->utc_mon  = -1;
    r->utc_day  = -1;
    r->callback = NULL;
    //r->fix.size = sizeof(r->fix);   // ginger  
    r->fix.size = sizeof(GpsLocation);   // ginger  // 2011.02.16

    nmea_reader_update_utc_diff( r );
}


static void
nmea_reader_set_callback( NmeaReader*  r, gps_location_callback  cb )
{
    if(GPS_DEBUG) LOGI("%s:", __FUNCTION__);
    r->callback = cb;
    if (cb != NULL && r->fix.flags != 0) {
        LOGI("%s: sending latest fix to new callback", __FUNCTION__);
        r->callback( &r->fix );
        r->fix.flags = 0;
    }
}

#if GPS_SV_INCLUDE
static void
nmea_reader_set_sv_callback( NmeaReader*  r, gps_sv_status_callback  cb )
{
    r->sv_callback = cb;
    if (cb != NULL) {
        LOGI("%s: sending latest sv info to new callback", __FUNCTION__);
    }
}
#endif

static int
nmea_reader_update_time( NmeaReader*  r, Token  tok )
{
    int        hour, minute;
    double     seconds;
    struct tm  tm;
    time_t     fix_time;

    if(GPS_DEBUG) LOGI("%s:", __FUNCTION__);
    if (tok.p + 6 > tok.end)
        return -1;

    if (r->utc_year < 0) {
        // no date yet, get current one
        time_t  now = time(NULL);
        gmtime_r( &now, &tm );
        r->utc_year = tm.tm_year + 1900;
        r->utc_mon  = tm.tm_mon + 1;
        r->utc_day  = tm.tm_mday;
    }

	nmea_reader_update_utc_diff(r);    //update r->utc_diff when change time zone

    hour    = str2int(tok.p,   tok.p+2);
    minute  = str2int(tok.p+2, tok.p+4);
    seconds = str2float(tok.p+4, tok.end);

    tm.tm_hour  = hour;
    tm.tm_min   = minute;
    tm.tm_sec   = (int) seconds;
    tm.tm_year  = r->utc_year - 1900;
    tm.tm_mon   = r->utc_mon - 1;
    tm.tm_mday  = r->utc_day;
    tm.tm_isdst = -1;

    fix_time = mktime( &tm ) - r->utc_diff;
    r->fix.timestamp = (long long)fix_time * 1000;
    return 0;
}

static int
nmea_reader_update_date( NmeaReader*  r, Token  date, Token  time )
{
    Token  tok = date;
    int    day, mon, year;

    if(GPS_DEBUG) LOGI("%s:", __FUNCTION__);
    if (tok.p + 6 != tok.end) {
        LOGI("date not properly formatted: '%.*s'", tok.end-tok.p, tok.p);
        return -1;
    }
    day  = str2int(tok.p, tok.p+2);
    mon  = str2int(tok.p+2, tok.p+4);
    year = str2int(tok.p+4, tok.p+6) + 2000;

    if ((day|mon|year) < 0) {
        LOGI("date not properly formatted: '%.*s'", tok.end-tok.p, tok.p);
        return -1;
    }

    r->utc_year  = year;
    r->utc_mon   = mon;
    r->utc_day   = day;

    return nmea_reader_update_time( r, time );
}


static double
convert_from_hhmm( Token  tok )
{
    double  val     = str2float(tok.p, tok.end);
    int     degrees = (int)(floor(val) / 100);
    double  minutes = val - degrees*100.;
    double  dcoord  = degrees + minutes / 60.0;

    if(GPS_DEBUG) LOGI("%s:", __FUNCTION__);	
    return dcoord;
}


static int
nmea_reader_update_latlong( NmeaReader*  r,
                            Token        latitude,
                            char         latitudeHemi,
                            Token        longitude,
                            char         longitudeHemi )
{
    double   lat, lon;
    Token    tok;

    if(GPS_DEBUG) LOGI("%s:", __FUNCTION__);
    tok = latitude;
    if (tok.p + 6 > tok.end) {
        if(GPS_DEBUG) LOGI("latitude is too short: '%.*s'", tok.end-tok.p, tok.p);
        return -1;
    }
    lat = convert_from_hhmm(tok);
    if (latitudeHemi == 'S')
        lat = -lat;

    tok = longitude;
    if (tok.p + 6 > tok.end) {
        LOGI("longitude is too short: '%.*s'", tok.end-tok.p, tok.p);
        return -1;
    }
    lon = convert_from_hhmm(tok);
    if (longitudeHemi == 'W')
        lon = -lon;

    r->fix.flags    |= GPS_LOCATION_HAS_LAT_LONG;
    r->fix.latitude  = lat;
    r->fix.longitude = lon;
    return 0;
}


static int
nmea_reader_update_altitude( NmeaReader*  r,
                             Token        altitude,
                             Token        units )
{
    double  alt;
    Token   tok = altitude;

    if(GPS_DEBUG) LOGI("%s:", __FUNCTION__);
    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GPS_LOCATION_HAS_ALTITUDE;
    r->fix.altitude = str2float(tok.p, tok.end);
    return 0;
}

static int
nmea_reader_update_accuracy( NmeaReader*  r,
                             Token        accuracy )
{
    double  acc;
    Token   tok = accuracy;

    if (tok.p >= tok.end)
        return -1;

    r->fix.accuracy = str2float(tok.p, tok.end);

    if (r->fix.accuracy == 99.99){
        return 0;
    }

    r->fix.flags   |= GPS_LOCATION_HAS_ACCURACY;
    return 0;
}

static int
nmea_reader_update_bearing( NmeaReader*  r,
                            Token        bearing )
{
    double  alt;
    Token   tok = bearing;

    if(GPS_DEBUG) LOGI("%s:", __FUNCTION__);
    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GPS_LOCATION_HAS_BEARING;
    r->fix.bearing  = str2float(tok.p, tok.end);
    return 0;
}


static int
nmea_reader_update_speed( NmeaReader*  r,
                          Token        speed )
{
    double  alt;
    Token   tok = speed;

    if(GPS_DEBUG) LOGI("%s:", __FUNCTION__);
    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GPS_LOCATION_HAS_SPEED;

    //knot converted to m/s,mSpeed=1.852*nSpeed/3.6
    r->fix.speed = str2float(tok.p, tok.end)*1.852/3.6;

    return 0;
}


static void
nmea_reader_parse( NmeaReader*  r )
{
   /* we received a complete sentence, now parse it to generate
    * a new GPS fix...
    */
    NmeaTokenizer  tzer[1];
    Token          tok;

//  LOGI("%s:", __FUNCTION__);
    LOGI("Received: %.*s", r->pos, r->in);
    if (r->pos < 9) {
        LOGI("Too short. discarded.");
        return;
    }

    nmea_tokenizer_init(tzer, r->in, r->in + r->pos);
#if GPS_DEBUG
    {
        int  n;
        LOGI("Found %d tokens", tzer->count);
        for (n = 0; n < tzer->count; n++) {
            Token  tok = nmea_tokenizer_get(tzer,n);
            LOGI("%2d: '%.*s'", n, tok.end-tok.p, tok.p);
        }
    }
#endif

    tok = nmea_tokenizer_get(tzer, 0);
    if (tok.p + 5 > tok.end) {
        LOGI("sentence id '%.*s' too short, ignored.", tok.end-tok.p, tok.p);
        return;
    }

    // ignore first two characters.
    tok.p += 2;
    if ( !memcmp(tok.p, "GGA", 3) ) {
        // GPS fix
        Token  tok_time          = nmea_tokenizer_get(tzer,1);
        Token  tok_latitude      = nmea_tokenizer_get(tzer,2);
        Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer,3);
        Token  tok_longitude     = nmea_tokenizer_get(tzer,4);
        Token  tok_longitudeHemi = nmea_tokenizer_get(tzer,5);
        Token  tok_altitude      = nmea_tokenizer_get(tzer,9);
        Token  tok_altitudeUnits = nmea_tokenizer_get(tzer,10);

        nmea_reader_update_time(r, tok_time);
        nmea_reader_update_latlong(r, tok_latitude,
                                      tok_latitudeHemi.p[0],
                                      tok_longitude,
                                      tok_longitudeHemi.p[0]);
        nmea_reader_update_altitude(r, tok_altitude, tok_altitudeUnits);

    } else if ( !memcmp(tok.p, "GSA", 3) ) {
#if GPS_SV_INCLUDE

        Token  tok_fixStatus   = nmea_tokenizer_get(tzer, 2);
        int i;

        if (tok_fixStatus.p[0] != '\0' && tok_fixStatus.p[0] != '1') {

            Token  tok_accuracy      = nmea_tokenizer_get(tzer, 15);

            nmea_reader_update_accuracy(r, tok_accuracy);

            r->sv_status.used_in_fix_mask = 0ul;

            for (i = 3; i <= 14; ++i){

                Token  tok_prn  = nmea_tokenizer_get(tzer, i);
                int prn = str2int(tok_prn.p, tok_prn.end);

                if(GPS_DEBUG) LOGI("%s: prn is %d", __FUNCTION__, r->sv_status.used_in_fix_mask);
                if (prn > 0){
                    //r->sv_status.used_in_fix_mask |= (1ul << (32 - prn));
                    r->sv_status.used_in_fix_mask |= (1ul << (prn-1));
                    r->sv_status_changed = 1;
                    if(GPS_DEBUG) LOGI("%s: fix mask is 0x%08X", __FUNCTION__, r->sv_status.used_in_fix_mask);
                }

            }

        }
#endif
        // do something ?
    } else if ( !memcmp(tok.p, "RMC", 3) ) {
        Token  tok_time          = nmea_tokenizer_get(tzer,1);
        Token  tok_fixStatus     = nmea_tokenizer_get(tzer,2);
        Token  tok_latitude      = nmea_tokenizer_get(tzer,3);
        Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer,4);
        Token  tok_longitude     = nmea_tokenizer_get(tzer,5);
        Token  tok_longitudeHemi = nmea_tokenizer_get(tzer,6);
        Token  tok_speed         = nmea_tokenizer_get(tzer,7);
        Token  tok_bearing       = nmea_tokenizer_get(tzer,8);
        Token  tok_date          = nmea_tokenizer_get(tzer,9);

        if(GPS_DEBUG) LOGI("in RMC, fixStatus=%c", tok_fixStatus.p[0]);
        if (tok_fixStatus.p[0] == 'A')
        {
            nmea_reader_update_date( r, tok_date, tok_time );

            nmea_reader_update_latlong( r, tok_latitude,
                                           tok_latitudeHemi.p[0],
                                           tok_longitude,
                                           tok_longitudeHemi.p[0] );

            nmea_reader_update_bearing( r, tok_bearing );
            nmea_reader_update_speed  ( r, tok_speed );
        }
    } else if ( !memcmp(tok.p, "GSV", 3) ) {
#if GPS_SV_INCLUDE
        Token  tok_noSatellites  = nmea_tokenizer_get(tzer, 3);
        int    noSatellites = str2int(tok_noSatellites.p, tok_noSatellites.end);

        if (noSatellites > 0) {

            Token  tok_noSentences   = nmea_tokenizer_get(tzer, 1);
            Token  tok_sentence      = nmea_tokenizer_get(tzer, 2);

            int sentence = str2int(tok_sentence.p, tok_sentence.end);
            int totalSentences = str2int(tok_noSentences.p, tok_noSentences.end);
            int curr;
            int i;

            if (sentence == 1) {
                r->sv_status_changed = 0;
                r->sv_status.num_svs = 0;
            }

            curr = r->sv_status.num_svs;

            i = 0;

            while (i < 4 && r->sv_status.num_svs < noSatellites){

                Token  tok_prn = nmea_tokenizer_get(tzer, i * 4 + 4);
                Token  tok_elevation = nmea_tokenizer_get(tzer, i * 4 + 5);
                Token  tok_azimuth = nmea_tokenizer_get(tzer, i * 4 + 6);
                Token  tok_snr = nmea_tokenizer_get(tzer, i * 4 + 7);

                r->sv_status.sv_list[curr].prn = str2int(tok_prn.p, tok_prn.end);
                r->sv_status.sv_list[curr].elevation = str2float(tok_elevation.p, tok_elevation.end);
                r->sv_status.sv_list[curr].azimuth = str2float(tok_azimuth.p, tok_azimuth.end);
                r->sv_status.sv_list[curr].snr = str2float(tok_snr.p, tok_snr.end);

                r->sv_status.num_svs += 1;

                curr += 1;

                i += 1;
            }

            if (sentence == totalSentences) {
                r->sv_status_changed = 1;
            }
        }

        if(GPS_DEBUG) LOGI("%s: GSV message with total satellites %d", __FUNCTION__, noSatellites);   
#endif
    } else {
        tok.p -= 2;
        if(GPS_DEBUG) LOGI("unknown sentence '%.*s", tok.end-tok.p, tok.p);
    }
    if (r->fix.flags != 0) {
#if GPS_DEBUG
        char   temp[256];
        char*  p   = temp;
        char*  end = p + sizeof(temp);
        struct tm   utc;

        p += snprintf( p, end-p, "sending fix" );
        if (r->fix.flags & GPS_LOCATION_HAS_LAT_LONG) {
            p += snprintf(p, end-p, " lat=%g lon=%g", r->fix.latitude, r->fix.longitude);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_ALTITUDE) {
            p += snprintf(p, end-p, " altitude=%g", r->fix.altitude);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_SPEED) {
            p += snprintf(p, end-p, " speed=%g", r->fix.speed);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_BEARING) {
            p += snprintf(p, end-p, " bearing=%g", r->fix.bearing);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_ACCURACY) {
            p += snprintf(p,end-p, " accuracy=%g", r->fix.accuracy);
        }
        gmtime_r( (time_t*) &r->fix.timestamp, &utc );
        p += snprintf(p, end-p, " time=%s", asctime( &utc ) );
//      LOGI(temp);
#endif
        if (r->callback) {
			LOGI("======dzwei, before calling location callback !");
            r->callback( &r->fix );
            r->fix.flags = 0;
        }
        else {
            LOGI("no callback, keeping data until needed !");
        }
#if GPS_SV_INCLUDE
        if ((r->sv_status_changed == 1) && (r->sv_callback)){
            r->sv_callback(&r->sv_status);
            r->sv_status_changed = 0;
        }
        else{
            LOGI("no sv callback, keeping data until needed !");
        }
#endif
    }
}


static void
nmea_reader_addc( NmeaReader*  r, int  c )
{
 //       LOGI("%s:", __FUNCTION__);
    if (r->overflow) {
        r->overflow = (c != '\n');
        return;
    }

    if (r->pos >= (int) sizeof(r->in)-1 ) {
        r->overflow = 1;
        r->pos      = 0;
        return;
    }

	// the buffer must has '$' as beginning
	if (r->pos == 0)
	{
		if (c != '$')
			return;
	}
	
    r->in[r->pos] = (char)c;
    r->pos       += 1;

    if (c == '\n') {
        nmea_reader_parse( r );
        r->pos = 0;
    }
}


/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       C O N N E C T I O N   S T A T E                 *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

/* commands sent to the gps thread */
enum {
    CMD_QUIT  = 0,
    CMD_START = 1,
    CMD_STOP  = 2
};


/* this is the state of our connection to the qemu_gpsd daemon */
typedef struct {
    int                     init;
    int                     fd;             // UART5용    FD
    #if GPS_GPIO_INCLUDE
    int                     fdGps;          // GPS_GPIO용 FD
    #endif
    GpsCallbacks            callbacks;
    pthread_t               thread;
    int                     control[2];
    char                    device[32];
    
    // QemuChannel             channel;     // We are not use QEMU
} GpsState;

static GpsState  _gps_state[1];

#if GPS_SV_INCLUDE
static char * gps_idle_on   = "$PCGDC,IDLEON,1,*1\r\n";
static char * gps_idle_off  = "$PCGDC,IDLEOFF,1,*1\r\n";
#endif
static void
gps_state_done( GpsState*  s )
{
    // tell the thread to quit, and wait for it
    char   cmd = CMD_QUIT;
    void*  dummy;

    if(GPS_DEBUG) LOGI("%s:", __FUNCTION__);
    write( s->control[0], &cmd, 1 );
    pthread_join(s->thread, &dummy);

    // close the control socket pair
    close( s->control[0] ); s->control[0] = -1;
    close( s->control[1] ); s->control[1] = -1;

    // close connection to the QEMU GPS daemon
    close( s->fd ); s->fd = -1;                                 // UART 5

    #if GPS_GPIO_INCLUDE
    close( s->fdGps ); s->fdGps = -1;                           // GPS_GPIO
    #endif
    s->init = 0;
}


static void
gps_state_start( GpsState*  s )
{
    char  cmd = CMD_START;
    int   ret, dwByteWrite;
	unsigned char start_gps[7] = {0xFA, 0xFA, 0x07, 0x00, 0x92, 0x8D, 0x02};

    if(GPS_DEBUG) LOGI("%s:", __FUNCTION__);
    do { ret=write( s->control[0], &cmd, 1 ); }
    while (ret < 0 && errno == EINTR);

    if (ret != 1)
        LOGI("%s: could not send CMD_START command: ret=%d: %s",
          __FUNCTION__, ret, strerror(errno));

#if GPS_SV_INCLUDE
    write(s->fd,gps_idle_off,strlen(gps_idle_off));
    if(GPS_DEBUG) LOGI("%s",gps_idle_off);

    #if GPS_GPIO_INCLUDE
    ret = ioctl(s->fdGps, 0); // 0 -> On, 1 -> Off              // bonovo_GPS
    #endif
    //GPS_GPIO드라이버의 IOCTL을 호출, GPS모듈을 ON/OFF함
#endif

	dwByteWrite = write(s->fd, start_gps, 7);
	if(GPS_DEBUG) LOGI("%d\n",dwByteWrite);

}


static void
gps_state_stop( GpsState*  s )
{
    char  cmd = CMD_STOP;
    int   ret, dwByteWrite;
	unsigned char stop_gps[7] = {0xFA, 0xFA, 0x07, 0x00, 0x93, 0x8E,0x02};

    if(GPS_DEBUG) LOGI("%s:", __FUNCTION__);
    do { ret=write( s->control[0], &cmd, 1 ); }
    while (ret < 0 && errno == EINTR);

    if (ret != 1)
        LOGI("%s: could not send CMD_STOP command: ret=%d: %s",
          __FUNCTION__, ret, strerror(errno));

#if GPS_SV_INCLUDE
    write(s->fd,gps_idle_on,strlen(gps_idle_on));
    if(GPS_DEBUG) LOGI("%s",gps_idle_on);

    #if GPS_GPIO_INCLUDE
    ret = ioctl(s->fdGps, 1); // 0 -> On, 1 -> Off              // bonovo_GPS
    #endif
    //call IOCTL of GPS_GPIO Driver, It is GPS Moudle on/off
#endif

	dwByteWrite = write(s->fd, stop_gps, 7);
	if(GPS_DEBUG) LOGI("%d\n",dwByteWrite);
}


static int
epoll_register( int  epoll_fd, int  fd )
{
    struct epoll_event  ev;
    int                 ret, flags;

    if(GPS_DEBUG) LOGI("%s:", __FUNCTION__);
    /* important: make the fd non-blocking */
    flags = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    ev.events  = EPOLLIN;
    ev.data.fd = fd;
    do {
        ret = epoll_ctl( epoll_fd, EPOLL_CTL_ADD, fd, &ev );
    } while (ret < 0 && errno == EINTR);
    return ret;
}


static int
epoll_deregister( int  epoll_fd, int  fd )
{
    int  ret;

	if(GPS_DEBUG) LOGI("%s:", __FUNCTION__);
    do {
        ret = epoll_ctl( epoll_fd, EPOLL_CTL_DEL, fd, NULL );
    } while (ret < 0 && errno == EINTR);
    return ret;
}

/* this is the main thread, it waits for commands from gps_state_start/stop and,
 * when started, messages from the QEMU GPS daemon. these are simple NMEA sentences
 * that must be parsed to be converted into GPS fixes sent to the framework
 */
static void
gps_state_thread( void*  arg )
{
    GpsState*   state = (GpsState*) arg;
    NmeaReader  reader[1];
    int         epoll_fd   = epoll_create(2);
    int         started    = 0;
    int         gps_fd     = state->fd;
    int         control_fd = state->control[1];

    if(GPS_DEBUG) LOGI("%s:epoll_fd=%d", __FUNCTION__,epoll_fd);
    nmea_reader_init( reader );

    // register control file descriptors for polling
    epoll_register( epoll_fd, control_fd );
    epoll_register( epoll_fd, gps_fd );

    if(GPS_DEBUG) LOGI("gps thread running");
    // now loop
    for (;;) {
        struct epoll_event   events[2];
        int                  ne, nevents;

        nevents = epoll_wait( epoll_fd, events, 2, -1 );
        if (nevents < 0) {
            if (errno != EINTR)
                LOGE("epoll_wait() unexpected error: %s", strerror(errno));
            continue;
        }
        if(GPS_DEBUG) LOGI("gps thread received %d events", nevents);
        for (ne = 0; ne < nevents; ne++) {
            if ((events[ne].events & (EPOLLERR|EPOLLHUP)) != 0) {
                LOGE("EPOLLERR or EPOLLHUP after epoll_wait() !?");
                goto Exit;
            }
            if ((events[ne].events & EPOLLIN) != 0) {
                int  fd = events[ne].data.fd;

                if (fd == control_fd)
                {
                    char  cmd = 255;
                    int   ret;
                    if(GPS_DEBUG) LOGI("gps control fd event");
                    do {
                        ret = read( fd, &cmd, 1 );
                    } while (ret < 0 && errno == EINTR);

                    if(GPS_DEBUG) LOGI("cmd [%d]", cmd);

                    if (cmd == CMD_QUIT) {
                        LOGI("gps thread quitting on demand");
                        goto Exit;
                    }
                    else if (cmd == CMD_START) {
                        if (!started) {
                            LOGI("gps thread starting  location_cb=%p", state->callbacks.location_cb);
                            started = 1;
                            nmea_reader_set_callback( reader, state->callbacks.location_cb );
#if GPS_SV_INCLUDE
                            nmea_reader_set_sv_callback( reader, state->callbacks.sv_status_cb );
#endif
                        }
                    }
                    else if (cmd == CMD_STOP) {
                        if (started) {
                            LOGI("gps thread stopping");
                            started = 0;
                            nmea_reader_set_callback( reader, NULL );
#if GPS_SV_INCLUDE
                            nmea_reader_set_sv_callback( reader, NULL );
#endif
                        }
                    }
                }
                else if (fd == gps_fd)
                {
                    char  buff[256];
                    if(GPS_DEBUG) LOGI("gps fd event");
                    for (;;) {
                        int  nn, ret;

                        ret = read( fd, buff, sizeof(buff) );
                        if (ret < 0) {
                            if (errno == EINTR)
                                continue;
                            if (errno != EWOULDBLOCK)
                                LOGE("error while reading from gps daemon socket: %s:", strerror(errno));
                            break;
                        }
                        if (ret == 0)	//add by bonovo
                        {
                            break;
                        }
                        if(ret > 0)
                            if(GPS_DEBUG) LOGI("received %d bytes: %.*s", ret, ret, buff);
                        for (nn = 0; nn < ret; nn++)
                            nmea_reader_addc( reader, buff[nn] );
                    }
                    if(GPS_DEBUG) LOGI("gps fd event end");
                }
                else
                {
                    LOGE("epoll_wait() returned unkown fd %d ?", fd);
                }
            }
        }
    }
Exit:
    epoll_deregister( epoll_fd, control_fd );
    epoll_deregister( epoll_fd, gps_fd );
    close(epoll_fd);
    return NULL;
}

static int
bonovo_channel_open_tty( GpsState*      state)
{
    char   key[PROPERTY_KEY_MAX];
    char   prop[PROPERTY_VALUE_MAX];
    int    ret;

    if(GPS_DEBUG) LOGI("%s:", __FUNCTION__);
/*    ret = snprintf(key, sizeof key, "ro.kernel.android.gps");
    if (ret >= (int)sizeof key)
        return -1;

    if (property_get(key, prop, "") == 0) {
        LOGI("no kernel-provided %s device name", "ro.kernel.android.gps");
        return -1;
    }

    ret = snprintf(state->device, sizeof state->device,
                    "/dev/%s", prop);
    if (ret >= (int)sizeof state->device) {
        LOGI("%s device name too long: '%s'", "ro.kernel.android.gps", prop);
        return -1;
    }*/
		ret = snprintf(state->device, sizeof state->device,
                    "/dev/%s", "ttyS3");
    return 0;
}


static void gps_state_init( GpsState*  state, GpsCallbacks* callbacks ) // 2011.02.16
{

    char   key[PROPERTY_KEY_MAX];
    char   prop[PROPERTY_VALUE_MAX];

    if(GPS_DEBUG) LOGI("%s:", __FUNCTION__);
    state->init       =  1;
    state->control[0] = -1;
    state->control[1] = -1;
    state->fd         = -1;
    #if GPS_GPIO_INCLUDE
    state->fdGps      = -1;
    #endif
 
    if( bonovo_channel_open_tty(state) < 0 ) {
        LOGI("bonovo :fail! set bonovo_channel_open_tty()");
        return;
    }
    
    state->fd = open(state->device, O_RDWR | O_NONBLOCK | O_NOCTTY);         // UART5
    if(GPS_DEBUG) LOGI("bonovo : %s Device Open FDescriptor %d", state->device, state->fd);

    if (state->fd < 0) {
        LOGI("bonovo : no gps Hardware detected");
        return;
    }

    if(GPS_DEBUG) LOGI("uart open %s success!", state->device);

    #if GPS_GPIO_INCLUDE
    state->fdGps = open("/dev/gps_gpio", O_RDWR);                               // Gps_GPIO
    if(GPS_DEBUG) LOGI("bonovo : Gps_GPIO Device Open FDescriptor %d",state->fdGps);

    if (state->fdGps < 0) {
        LOGI("bonovo : Couldn't open gps_gpio");
        return;
    }
    #endif
    
    if(GPS_DEBUG) LOGI("gps will read from %s", state->device);

    if ( socketpair( AF_LOCAL, SOCK_STREAM, 0, state->control ) < 0 ) {
        LOGE("could not create thread control socket pair: %s", strerror(errno));
        goto Fail;
    }

    //if ( pthread_create( &state->thread, NULL, gps_state_thread, state ) != 0 ) {
    //    LOGE("could not create gps thread: %s", strerror(errno));
    //    goto Fail;
    //}
    state->thread = callbacks->create_thread_cb("loc_api", gps_state_thread, state);  // 2011.02.16

    if(GPS_DEBUG) LOGI("gps state initialized");
    return;

Fail:
    gps_state_done( state );
}


/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       I N T E R F A C E                               *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/


static int
bonovo_gps_init(GpsCallbacks* callbacks)
{
    GpsState*  s = _gps_state;

    if(GPS_DEBUG) LOGI("%s:", __FUNCTION__);
    if (!s->init)
        gps_state_init(s, callbacks);  // 2011.02.16

    if (s->fd < 0)
        return -1;

    s->callbacks = *callbacks;
    if(GPS_DEBUG) LOGI("%s: called", __FUNCTION__);                      // for Debug

    return 0;
}

static void
bonovo_gps_cleanup(void)
{
    GpsState*  s = _gps_state;

    if(GPS_DEBUG) LOGI("%s:", __FUNCTION__);
    if (s->init)
        gps_state_done(s);
    if(GPS_DEBUG) LOGI("%s: called", __FUNCTION__);                      // for Debug
}


static int
bonovo_gps_start()
{
    GpsState*  s = _gps_state;

    if(GPS_DEBUG) LOGI("%s:", __FUNCTION__);
    if (!s->init) {
        LOGI("%s: called with uninitialized state !!", __FUNCTION__);
        return -1;
    }

    if(GPS_DEBUG) LOGI("%s: called", __FUNCTION__);
    gps_state_start(s);
    return 0;
}


static int
bonovo_gps_stop()
{
    GpsState*  s = _gps_state;

    if(GPS_DEBUG) LOGI("%s:", __FUNCTION__);
    if (!s->init) {
        LOGI("%s: called with uninitialized state !!", __FUNCTION__);
        return -1;
    }

    if(GPS_DEBUG) LOGI("%s: called", __FUNCTION__);
    gps_state_stop(s);
    return 0;
}

static int
bonovo_gps_inject_time(GpsUtcTime time, int64_t timeReference, int uncertainty)
{
    if(GPS_DEBUG) LOGI("%s: called", __FUNCTION__);                          // for Debug
    return 0;
}

static int
bonovo_gps_inject_location(double latitude, double longitude, float accuracy)
{
    if(GPS_DEBUG) LOGI("%s: called", __FUNCTION__);                          // for Debug
    return 0;
}


static void
bonovo_gps_delete_aiding_data(GpsAidingData flags)
{
    if(GPS_DEBUG) LOGI("%s: called", __FUNCTION__);                          // for Debug
}

//static int bonovo_gps_set_position_mode(GpsPositionMode mode, int fix_frequency)
static int bonovo_gps_set_position_mode(GpsPositionMode mode, GpsPositionRecurrence recurrence,
                uint32_t min_interval, uint32_t preferred_accuracy, uint32_t preferred_time)  // 2011.02.16
{
    if(GPS_DEBUG) LOGI("%s: called", __FUNCTION__);
    // FIXME - support fix_frequency
    // only standalone supported for now.

    if (mode != GPS_POSITION_MODE_STANDALONE) {
        if(GPS_DEBUG) LOGI("%s: called MODE_STANDALONE", __FUNCTION__);      // for Debug
        return -1;
    }
    return 0;
}

static const void*
bonovo_gps_get_extension(const char* name)
{
    // no extensions supported
    if(GPS_DEBUG) LOGI("%s:", __FUNCTION__);
    return NULL;
}

static const GpsInterface  bonovoGpsInterface = {
    sizeof(GpsInterface),
    bonovo_gps_init,
    bonovo_gps_start,
    bonovo_gps_stop,
    bonovo_gps_cleanup,
    bonovo_gps_inject_time,
    bonovo_gps_inject_location,
    bonovo_gps_delete_aiding_data,
    bonovo_gps_set_position_mode,
    bonovo_gps_get_extension,
};

const GpsInterface* gps__get_gps_interface(struct gps_device_t* dev)
{
    if(GPS_DEBUG) LOGI("%s: called\n", __FUNCTION__);
    return &bonovoGpsInterface;
}

static int open_gps(const struct hw_module_t* module, const char* name, struct hw_device_t** device)
{
    struct gps_device_t *dev = malloc(sizeof(struct gps_device_t));
    memset(dev, 0, sizeof(*dev));
    if(GPS_DEBUG) LOGI("%s: called\n", __FUNCTION__);
    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = 0;
    dev->common.module = (struct hw_module_t*)module;
//    dev->common.close = (int (*)(struct hw_device_t*))close_lights;
    dev->get_gps_interface = gps__get_gps_interface;

    *device = (struct hw_device_t*)dev;
    return 0;
}


static struct hw_module_methods_t gps_module_methods = {
    open: open_gps
};

struct gps_module_t HAL_MODULE_INFO_SYM = {
    common: {
        .tag = HARDWARE_MODULE_TAG,
	    .version_major = 1,
	    .version_minor = 0,
	    .id = GPS_HARDWARE_MODULE_ID,
	    .name = "Bonovo GPS Module",
	    .author = "The Android Open Source Project",
	    .methods = &gps_module_methods,
    }
};
