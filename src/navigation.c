#include "board.h"
#include "mw.h"

//#define Earthradius           637100078.5d                         // Average Earthradius in cm (from 637813700cm ellipsoid to a sphere)
//#define MagicEarthNumber       1.1119494f                          // Based on average Earthradius pi/180 * Earthradius / 10^7
//#define MagicEarthNumber       1.1113175f                            // used by apm "INERTIALNAV_LATLON_TO_CM"
#define MagicEarthNumber 1.113195f                                   // LOL! The "new" apm number
//#define MagicEarthNumber         1.11163345f                         // OWN Earth number does correct projection!!

// They are defined in mw.h
// #define LAT  0
// #define LON  1
// #define GPS_Y 0
// #define GPS_X 1

typedef struct PID_PARAM_
{
    float kP;
    float kI;
    float kD;
    float Imax;
    float Dmax;
} PID_PARAM;

typedef struct PID_
{
    float integrator;          // integrator value "I"
    float last_rateerror;      // D
    float lastDTerm;           // D
    float last_delta1;         // D
    float last_delta2;         // D
} PID;

// NAV & PH PID Variables
static PID_PARAM posholdPID_PARAM;
static PID_PARAM poshold_ratePID_PARAM;
static PID_PARAM navPID_PARAM;

static PID       posholdPID[2];
static PID       poshold_ratePID[2];
static PID       navPID[2];

// INS & Core Variables
static int16_t   maxbank100;              // Maximum GPS Tiltangle
static int16_t   maxbankbrake100;         // Maximum GPS Brake Tiltangle < maxbank100
static float     Real_GPS_speed[2] = { 0, 0 }; // Is the earthframespeed measured by GPS Coord Difference
static float     MIX_speed[2]      = { 0, 0 }; // That is a 1:1 Mix of Acc speed and GPS Earthframespeed
static int32_t   Last_Real_GPS_coord[2];
uint32_t         TimestampNewGPSdata;     // Crashpilot in micros

// PH Variables
static float     GpsPhAbsTub;             // Defines a "bathtub" around current PH Position, so the PosP will not brutally follow GPS wandering

// Earth / Location constants
static float     OneCmTo[2];              // Moves one cm in Gps coords
static float     CosLatScaleLon;          // this is used to offset the shrinking longitude as we go towards the poles
static float     GPSRAWtoRAD;

static float     get_P(float error, struct PID_PARAM_* pid);
static float     get_I(float error, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param);
static float     get_D(float error, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param);
static int32_t   wrap_36000(int32_t angle);


////////////////////////////////////////////////////////////////////////////////////
// Calculate our current speed vector from gps&acc position data
// This is another important part of the gps ins
void GPS_calc_velocity(void)                                                    // actual_speed[GPS_Y] y_GPS_speed positve = Up (NORTH) // actual_speed[GPS_X] x_GPS_speed positve = Right (EAST)
{
    static uint32_t LastTimestampNewGPSdata;
    static float    GPSmovementAdder[2];
    static bool     INSusable;
    float           gpsHz, tmp0;
    uint32_t        RealGPSDeltaTime;
    uint8_t         i;

    if (CosLatScaleLon == 0.0f) GPS_calc_longitude_scaling();                   // Init CosLatScaleLon if not already done to avoid div by zero etc..
    RealGPSDeltaTime = TimestampNewGPSdata - LastTimestampNewGPSdata;           // RealGPSDeltaTime in ms! NOT us!
    LastTimestampNewGPSdata = TimestampNewGPSdata;
    if (RealGPSDeltaTime != 0)                                                  // New GPS Data?
    {
        INSusable = false;                                                      // Set INS to ununsable in advance we will see later
        if (RealGPSDeltaTime < 400)                                             // In Time? 2,5Hz-XXHz
        {
            INSusable = true;                                                   // INS is alive
            gpsHz = 1000.0f/(float)RealGPSDeltaTime;                            // Set GPS Hz, try to filter below
            if (RealGPSDeltaTime >  10 && RealGPSDeltaTime <  30) gpsHz = 50.0f;// 50Hz Data  20ms filter out timejitter
            if (RealGPSDeltaTime >  40 && RealGPSDeltaTime <  60) gpsHz = 20.0f;// 20Hz Data  50ms filter out timejitter
            if (RealGPSDeltaTime >  80 && RealGPSDeltaTime < 120) gpsHz = 10.0f;// 10Hz Data 100ms filter out timejitter
            if (RealGPSDeltaTime > 180 && RealGPSDeltaTime < 220) gpsHz = 5.0f; //  5Hz Data 200ms
            if (RealGPSDeltaTime > 230 && RealGPSDeltaTime < 270) gpsHz = 4.0f; //  4Hz Data 250ms

            switch (cfg.gps_ins_mdl)                                            // GPS ins model.
            {
            case 1:                                                             // 1 = Based on lat/lon
            default:
                tmp0 = MagicEarthNumber * gpsHz;
                Real_GPS_speed[LON] = (float)(Real_GPS_coord[LON] - Last_Real_GPS_coord[LON]) * tmp0 * CosLatScaleLon ; // cm/s
                Real_GPS_speed[LAT] = (float)(Real_GPS_coord[LAT] - Last_Real_GPS_coord[LAT]) * tmp0;                   // cm/s
                break;
            case 2:                                                             // 2 = based on Groundcourse & Speed
                tmp0 = GPS_ground_course * RADX10;
                Real_GPS_speed[LON] = (float)GPS_speed * sinf(tmp0);            // cm/s
                Real_GPS_speed[LAT] = (float)GPS_speed * cosf(tmp0);
                break;
            }

            for (i = 0; i < 2; i++)
            {
                Last_Real_GPS_coord[i] = Real_GPS_coord[i];
                ACC_speed[i]           = (ACC_speed[i] * cfg.gps_ins_vel) + (Real_GPS_speed[i] * (1.0f - cfg.gps_ins_vel)); // CF: GPS Correction
                GPSmovementAdder[i]    = 0;                                     // This float accumulates the tiny acc movements between GPS reads
            }
        }
    }                                                                           // End of X Hz Loop

    if ((millis() - TimestampNewGPSdata) > 500) INSusable = false;              // INS is NOT OK, too long (500ms) no correction

    if (INSusable)
    {
        for (i = 0; i < 2; i++)
        {
            GPSmovementAdder[i] += (ACC_speed[i] * ACCDeltaTimeINS * OneCmTo[i]);
            GPS_coord[i]         = Real_GPS_coord[i] + (int32_t)GPSmovementAdder[i];// Bridge the time between GPS reads with acc data
            MIX_speed[i]         = (ACC_speed[i] + Real_GPS_speed[i]) * 0.5f;
        }
    }
    else
    {
        GPS_reset_nav();                                                        // Ins is fucked, reset stuff
        GPSmovementAdder[LAT] = GPSmovementAdder[LON] = 0;
    }
}

////////////////////////////////////////////////////////////////////////////////////
// Get distance between two points in cm Get bearing from pos1 to pos2, returns an 1deg = 100 precision
// Now with more correct BEARING calclation according to this: http://www.movable-type.co.uk/scripts/latlong.html
void GPS_distance_cm_bearing(int32_t * lat1, int32_t * lon1, int32_t * lat2, int32_t * lon2, uint32_t * dist, int32_t * bearing)
{
    float dLatRAW, dLonRAW, x, y, lat1RAD, lat2RAD, Coslat2RAD;
    if (*lat2 != 0 && *lat1 != 0 && *lon2 != 0 && *lon1 != 0)                   // Crashpilot Errorcheck
    {
        if (CosLatScaleLon == 0.0f) GPS_calc_longitude_scaling();
        dLatRAW    = (float)(*lat2 - *lat1);                                    // difference of latitude in 1/10 000 000 degrees
        dLonRAW    = (float)(*lon2 - *lon1);      
        x          = dLonRAW * CosLatScaleLon;
        *dist      = sqrtf(dLatRAW * dLatRAW + x * x) * MagicEarthNumber;       // dist in cm
        dLatRAW    = dLatRAW * GPSRAWtoRAD;                                     // 10^7 DEGdelta to Rad
        dLonRAW    = dLonRAW * GPSRAWtoRAD;
        lat1RAD    = *lat1   * GPSRAWtoRAD;
        lat2RAD    = *lat2   * GPSRAWtoRAD;
        Coslat2RAD = cosf(lat2RAD);
        y          = sinf(dLonRAW) * Coslat2RAD;
        x          = cosf(lat1RAD) * sinf(lat2RAD) - sinf(lat1RAD) * Coslat2RAD * cos(dLonRAW);
        *bearing   = constrain((int32_t)(atan2f(y, x) * RADtoDEG100), -18000, 18000);
        if (*bearing < 0) *bearing += 36000;
    }
    else                                                                        // Error!
    {
        *dist = 0;
        *bearing = 0;
    }
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate a location error between two gps coordinates. Crashpilot distance error in CM now!
void GPS_calc_location_error(int32_t *target_lat, int32_t *target_lng, int32_t *gps_lat, int32_t *gps_lng)
{
    if (CosLatScaleLon == 0.0f) GPS_calc_longitude_scaling();                   // Just in case scaling isn't done
    if (*target_lng != 0 && *target_lat != 0 && *gps_lng != 0 && *gps_lat != 0)
    {
        LocError[LON] = (float)(*target_lng - *gps_lng) * MagicEarthNumber * CosLatScaleLon; // X Error in cm not lon!
        LocError[LAT] = (float)(*target_lat - *gps_lat) * MagicEarthNumber;                  // Y Error in cm not lat!
    }
    else
    {
        LocError[LON] = 0;
        LocError[LAT] = 0;
    }
}

////////////////////////////////////////////////////////////////////////////////////
// Crashpilot NEW PH Logic
// #define GPS_X 1 // #define GPS_Y 0
// #define LON   1 // #define LAT   0
// VelEast;   // 1 // VelNorth;  // 0
// 0 is NICK part  // 1 is ROLL part
// actual_speed[GPS_Y] = VelNorth;
void GPS_calc_posholdCrashpilot(bool overspeed)
{
    uint8_t axis;
    float   p, i, d, rate_error, AbsPosErrorToVel, tmp0, tmp1;
    float   maxbank100new;

    for (axis = 0; axis < 2; axis++)
    {
        maxbank100new = maxbank100;
        if (overspeed)
        {
            tmp1 = abs(ACC_speed[axis]);
            if (tmp1 == 0) tmp1 = 1.0f;
            tmp0 = (float)cfg.gps_ph_settlespeed / tmp1;
            tmp1 = (float)cfg.gps_ph_minbrakepercent * 0.01f;                   // this is the minimal break percentage
            tmp0 = constrain(sqrt(tmp0), tmp1, 1.0f);                           // 1 - 100%
            maxbank100new = (float)maxbankbrake100 * tmp0;
        }
        tmp1 = LocError[axis];
        if (abs(tmp1) < cfg.gps_ph_abstub && cfg.gps_ph_abstub != 0)            // Keep linear Stuff beyond x cm
        {
            if (tmp1 < 0) tmp0 = -GpsPhAbsTub;                                  // Do flatter response below x cm
             else  tmp0 = GpsPhAbsTub;
             tmp1 = tmp1 * tmp1 * tmp0;                                         // Get a peace around current position
        }
        AbsPosErrorToVel = get_P(tmp1,                                       &posholdPID_PARAM);      // Do absolute position error here, that means transform positionerror to a velocityerror
        rate_error = AbsPosErrorToVel - ACC_speed[axis];                                              // Calculate Rate Error for actual axis
        rate_error = constrain(rate_error, -1000, 1000);                                              // +- 10m/s
        p          = get_P(rate_error,                                       &poshold_ratePID_PARAM);
        i          = get_I(AbsPosErrorToVel, &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM); // Constrained to half max P
        d          = get_D(rate_error      , &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM); // Constrained to half max P
        nav[axis]  = constrain(p + i + d, -maxbank100new, maxbank100new);
    }
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate the desired nav_lat and nav_lon for distance flying such as RTH
void GPS_calc_nav_rate(uint16_t max_speed)
{
    float   trig[2], rate_error;
    float   temp, tiltcomp, trgtspeed;
    uint8_t axis;
    int32_t crosstrack_error;
    if ((abs(wrap_18000(target_bearing - original_target_bearing)) < 4500) && cfg.nav_ctrkgain != 0)// If we are too far off or too close we don't do track following
    {
        temp = (float)(target_bearing - original_target_bearing) * RADX100;
        crosstrack_error = sinf(temp) * (float)wp_distance * cfg.nav_ctrkgain;  // Meters we are off track line
        nav_bearing = target_bearing + constrain(crosstrack_error, -3000, 3000);
        nav_bearing = wrap_36000(nav_bearing);
    }
    else nav_bearing = target_bearing;

    temp = (float)(9000l - nav_bearing) * RADX100;                              // nav_bearing and maybe crosstrack
    trig[GPS_X] = cosf(temp);
    trig[GPS_Y] = sinf(temp);
    for (axis = 0; axis < 2; axis++)
    {
        trgtspeed  = (trig[axis] * (float)max_speed);                           // Target speed
        rate_error = trgtspeed - MIX_speed[axis];                               // Since my INS Stuff is shit, reduce ACC influence to 50% anyway better than leadfilter
        rate_error = constrain(rate_error, -1000, 1000);
        nav[axis]  = get_P(rate_error,                        &navPID_PARAM) +  // P + I + D
                     get_I(rate_error, &dTnav, &navPID[axis], &navPID_PARAM) +
                     get_D(rate_error, &dTnav, &navPID[axis], &navPID_PARAM);
        if (cfg.nav_tiltcomp != 0)                                              // Do the apm 2.9.1 magic tiltcompensation
        {
            tiltcomp = trgtspeed * trgtspeed * ((float)cfg.nav_tiltcomp * 0.0001f);
            if(trgtspeed < 0) tiltcomp = -tiltcomp;
        }
        else tiltcomp = 0;
        nav[axis]  = constrain(nav[axis] + tiltcomp, -maxbank100, maxbank100);
        poshold_ratePID[axis].integrator = navPID[axis].integrator;
    }
}

uint16_t GPS_calc_desired_speed(void)
{
    uint16_t max = cfg.nav_speed_max;
    if (!WP_Fastcorner) max = (uint16_t)min((uint32_t)max, wp_distance / (uint32_t)cfg.nav_approachdiv); //nav_approachdiv = 2-10
    else max = min(max, wp_distance);
    if (max > waypoint_speed_gov)
    {
        waypoint_speed_gov += (100.0f * dTnav);                                 // increase speed
        max = waypoint_speed_gov;
    }
    max = constrain(max, (uint16_t)cfg.nav_speed_min, cfg.nav_speed_max);       // Put output in desired range
    return max;
}

////////////////////////////////////////////////////////////////////////////////////
// ***   Utilities   ***
////////////////////////////////////////////////////////////////////////////////////
void GPS_set_pids(void)                                                         // Is done after EEPROM read
{
    float imax = (float)cfg.gps_maxangle * 50;                                  // Set Imax to half maximal Tiltangle (*100)

    posholdPID_PARAM.kP        = (float)cfg.P8[PIDPOS]  /  100.0f;              // Original Scale
    posholdPID_PARAM.kI        = (float)cfg.I8[PIDPOS]  / 1000.0f;              // Now used
    posholdPID_PARAM.kD        = (float)cfg.D8[PIDPOS]  / 3000.0f;              // Not used but initialized
    posholdPID_PARAM.Imax      = imax;
    posholdPID_PARAM.Dmax      = imax;                                          // Set Dmax to imax for now
  
    poshold_ratePID_PARAM.kP   = (float)cfg.P8[PIDPOSR] /      5.0f;            // Need more P
    poshold_ratePID_PARAM.kI   = (float)cfg.I8[PIDPOSR] / 100000.0f;            // "I" is evil, leads to circeling scaled further down (/100)
    poshold_ratePID_PARAM.kD   = (float)cfg.D8[PIDPOSR] /   3000.0f;
    poshold_ratePID_PARAM.Imax = imax;
    poshold_ratePID_PARAM.Dmax = imax;                                          // Set Dmax to imax for now

    navPID_PARAM.kP            = (float)cfg.P8[PIDNAVR] /   10.0f;
    navPID_PARAM.kI            = (float)cfg.I8[PIDNAVR] / 1000.0f;
    navPID_PARAM.kD            = (float)cfg.D8[PIDNAVR] / 3000.0f;
    navPID_PARAM.Imax          = imax;
    navPID_PARAM.Dmax          = imax;                                          // Set Dmax to imax for now
    
    GPSRAWtoRAD     = 0.0000001f * M_PI / 180.0f;
    OneCmTo[LAT]    = 1.0f / MagicEarthNumber;                                  // Moves North one cm
    maxbank100      = (int16_t)cfg.gps_maxangle * 100;                          // Initialize some values here
    maxbankbrake100 = (int16_t)cfg.gps_ph_brakemaxangle * 100;
    if (maxbankbrake100 > maxbank100) maxbankbrake100 = maxbank100;
    if (cfg.gps_ph_abstub == 0) GpsPhAbsTub = 0;                                // Example for 300 cm: 0.01f * 1/300 * x * x
     else GpsPhAbsTub = (1.0f/(float)cfg.gps_ph_abstub);
}

static float get_P(float error, struct PID_PARAM_* pid)
{
    return error * pid->kP;
}

static float get_I(float error, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param)
{
    float time = *dt;
    pid->integrator += (error * pid_param->kI * time);
    pid->integrator  = constrain(pid->integrator, -pid_param->Imax, pid_param->Imax);
    return pid->integrator;
}

static float get_D(float error, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param)
{
    float delta, deltaSum;
    float time = *dt;
    delta               = (error - pid->last_rateerror) / time;
    pid->last_rateerror = error;
    deltaSum            = pid->last_delta1 + pid->last_delta2 + delta;                   // Moving average
    pid->last_delta2    = pid->last_delta1;
    pid->last_delta1    = delta;
    deltaSum            = pid->lastDTerm + (time / (GPSDpt1freqCut + time)) * (deltaSum - pid->lastDTerm);
    pid->lastDTerm      = deltaSum;
    deltaSum            = constrain(deltaSum * pid_param->kD, -pid_param->Dmax, pid_param->Dmax);
    return deltaSum;
}

static void reset_PID(struct PID_* pid)
{
    pid->integrator     = 0;
    pid->last_rateerror = 0;
    pid->lastDTerm      = 0;
    pid->last_delta1    = 0;
    pid->last_delta2    = 0;
}

void GPS_reset_home_position(void)
{
    if (f.GPS_FIX && GPS_numSat >= 5)
    {
        GPS_calc_longitude_scaling();
        GPS_home[LAT] = Real_GPS_coord[LAT];
        GPS_home[LON] = Real_GPS_coord[LON];
        nav_takeoff_heading = heading;                                          // save takeoff heading
        f.GPS_FIX_HOME = 1;
    }
}

void GPS_reset_nav(void)                                                        // reset navigation (stop the navigation processor, and clear nav)
{
    uint8_t i;
    for (i = 0; i < 2; i++)
    {
        GPS_coord[i] = Real_GPS_coord[i];                                  // Discard INS GPS pos and use the real
        Last_Real_GPS_coord[i] = Real_GPS_coord[i];
        ACC_speed[i] = MIX_speed[i] = Real_GPS_speed[i] = 0;
        LocError[i]  = 0;
        GPS_angle[i] = 0;
        nav_rated[i] = 0;
        nav[i]       = 0;
        reset_PID(&posholdPID[i]);
        reset_PID(&poshold_ratePID[i]);
        reset_PID(&navPID[i]);
    }
    waypoint_speed_gov = (float)cfg.nav_speed_min;
    WP_Fastcorner = false;
}

bool DoingGPS(void)
{
    if (f.GPS_HOLD_MODE || f.GPS_HOME_MODE) return true;
    else return false;
}

bool check_missed_wp(void)
{
    float temp;
    temp = (float)(target_bearing - original_target_bearing);
    temp = wrap_18000(temp);
    return (abs(temp) > 10000);                                                 // we passed the waypoint by 100 degrees
}

void GPS_calc_longitude_scaling(void)
{
    float rads = (float)Real_GPS_coord[LAT] * GPSRAWtoRAD;
    rads = fabs(rads);
    CosLatScaleLon = cosf(rads);                                                // can only be 0 at 90 degree, perhaps at the poles?
    if (CosLatScaleLon == 0) CosLatScaleLon = 0.001745328f;                     // Avoid divzero (value is cos of 89.9 Degree)
    OneCmTo[LON] = OneCmTo[LAT] / CosLatScaleLon;                               // Moves EAST  one cm // OneCmTo[LAT] calculated on startup
}

float wrap_18000(float value)
{
    while (value >  18000) value -= 36000;
    while (value < -18000) value += 36000;
    return value;
}

//static float wrap_1800(float value)
//{
//    while (value >  1800) value -= 3600;
//    while (value < -1800) value += 3600;
//    return value;
//}

static int32_t wrap_36000(int32_t angle)
{
    while (angle > 36000) angle -= 36000;
    while (angle <     0) angle += 36000;
    return angle;
}
