#include "board.h"
#include "mw.h"
#include "baseflight_mavlink.h"

typedef enum
{
    VAR_UINT8,
    VAR_INT8,
    VAR_UINT16,
    VAR_INT16,
    VAR_UINT32,
    VAR_FLOAT
} vartype_e;

typedef struct
{
    const char *name;
    const uint8_t type; // vartype_e
    void *ptr;
    const int32_t min;
    const int32_t max;
    const uint8_t lcd; // 1 = Displayed in LCD // 0 = Not displayed
} clivalue_t;

// we unset this on 'exit'
extern uint8_t cliMode;
static void cliAuxset(char *cmdline);
static void cliCMix(char *cmdline);
#ifdef debugmode
static void cliDebug(char *cmdline);
#endif
static void cliDefault(char *cmdline);
static void cliDump(char *cmdLine);
static void cliExit(char *cmdline);
static void cliFeature(char *cmdline);
static void cliHelp(char *cmdline);
static void cliMap(char *cmdline);
static void cliMixer(char *cmdline);
static void cliSet(char *cmdline);
static void cliStatus(char *cmdline);
static void cliVersion(char *cmdline);
static void cliScanbus(char *cmdline);
static void cliPassgps(char *cmdline);
static void cliFlash(char *cmdline);
static void cliErrorMessage(void);
static void cliSetVar(const clivalue_t *var, const int32_t value);
static void changeval(const clivalue_t *var, const int8_t adder);
static void cliPrintVar(const clivalue_t *var, uint32_t full);
static void cliGpslog(char *cmdline);

// serial LCD
static void LCDinit(void);
static void LCDoff(void);
static void LCDclear(void);
static void LCDline1(void);
static void LCDline2(void);

// OLED-Display
extern bool i2cLCD;                         // true, if an OLED-Display is connected

// from sensors.c
extern uint8_t batteryCellCount;
extern uint8_t accHardware;

// from config.c RC Channel mapping
extern const char rcChannelLetters[];

// gpspass
static uint8_t NewGPSByte;
static bool    HaveNewGpsByte;
static void    GPSbyteRec(uint16_t c);

// buffer
static char cliBuffer[48];
static uint32_t bufferIndex = 0;

static float _atof(const char *p);
static char *ftoa(float x, char *floatString);

// sync this with MultiType enum from mw.h
const char * const mixerNames[] =
{
    "TRI", "QUADP", "QUADX", "BI",
    "GIMBAL", "Y6", "HEX6",
    "FLYING_WING", "Y4", "HEX6X", "OCTOX8", "OCTOFLATP", "OCTOFLATX",
    "AIRPLANE", "HELI_120_CCPM", "HELI_90_DEG", "VTAIL4", "CUSTOM", NULL
};

// sync this with AvailableFeatures enum from board.h
const char * const featureNames[] =
{
    "PPM",
    "VBAT",
    "INFLIGHT_ACC_CAL",
    "SPEKTRUM",
    "GRAUPNERSUMH",
    "MOTOR_STOP",
    "SERVO_TILT",
    "LED",
    "GPS",
    "FAILSAFE",
    "SONAR",
    "PASS",
    "LCD",
    NULL
};

// sync this with AvailableSensors enum from board.h
const char * const sensorNames[] =
{
    "ACC", "BARO", "MAG", "SONAR", "GPS", NULL
};

//
const char * const accNames[] =
{
    "", "ADXL345", "MPU6050", "MMA845x", NULL
};

typedef struct
{
    char *name;
    char *param;
    void (*func)(char *cmdline);
} clicmd_t;

// should be sorted a..z for bsearch()
const clicmd_t cmdTable[] =
{
    { "auxset", "alternative to GUI", cliAuxset },
    { "cmix", "design custom mixer", cliCMix },
#ifdef debugmode
    { "debug", "show debugstuff", cliDebug },
#endif
    { "default", "load defaults & reboot", cliDefault },
    { "dump",    "dump config", cliDump },
    { "exit",    "exit & reboot", cliExit },
    { "feature", "list or -val or val", cliFeature },
    { "flash",   "flashmode", cliFlash },
    { "gpslog",  "dump gps log", cliGpslog },
    { "help",    "this text", cliHelp },
    { "map",     "mapping of rc channel order", cliMap },
    { "mixer",   "mixer name or list", cliMixer },
    { "passgps", "pass through gps data", cliPassgps },
    { "save",    "save and reboot", cliSave },
    { "scanbus", "scan i2c bus", cliScanbus },
    { "set",     "name=value or blank or * for list", cliSet },
    { "status",  "sys status & stats", cliStatus },
    { "version", "", cliVersion },
};
#define CMD_COUNT (sizeof(cmdTable) / sizeof(cmdTable[0]))

const clivalue_t valueTable[] =
{
    { "rc_db",                     VAR_UINT8,  &cfg.rc_db,                       0,         32, 1 },
    { "rc_dbyw",                   VAR_UINT8,  &cfg.rc_dbyw,                     0,        100, 1 },
    { "rc_dbah",                   VAR_UINT8,  &cfg.rc_dbah,                     1,        100, 1 },
    { "rc_dbgps",                  VAR_UINT8,  &cfg.rc_dbgps,                    0,        100, 1 },
    { "devorssi",                  VAR_UINT8,  &cfg.devorssi,                    0,          1, 0 },
    { "rssicut",                   VAR_UINT8,  &cfg.rssicut,                     0,         80, 0 },
    { "rc_mid",                    VAR_UINT16, &cfg.rc_mid,                   1200,       1700, 1 },
    { "rc_auxch",                  VAR_UINT8,  &cfg.rc_auxch,                    4,         10, 0 },
    { "rc_rate",                   VAR_UINT8,  &cfg.rcRate8,                     0,        250, 1 },
    { "rc_expo",                   VAR_UINT8,  &cfg.rcExpo8,                     0,        100, 1 },
    { "thr_mid",                   VAR_UINT8,  &cfg.thrMid8,                     0,        100, 1 },
    { "thr_expo",                  VAR_UINT8,  &cfg.thrExpo8,                    0,        250, 1 },
    { "roll_pitch_rate",           VAR_UINT8,  &cfg.rollPitchRate,               0,        100, 1 },
    { "yawrate",                   VAR_UINT8,  &cfg.yawRate,                     0,        100, 1 },
    { "esc_min",                   VAR_UINT16, &cfg.esc_min,                     0,       2000, 0 },
    { "esc_max",                   VAR_UINT16, &cfg.esc_max,                     0,       2000, 0 },
    { "esc_nfly",                  VAR_UINT16, &cfg.esc_nfly,                    0,       2000, 1 },
    { "esc_moff",                  VAR_UINT16, &cfg.esc_moff,                    0,       2000, 0 },
    { "esc_pwm",                   VAR_UINT16, &cfg.esc_pwm,                    50,        498, 0 },
    { "srv_pwm",                   VAR_UINT16, &cfg.srv_pwm,                    50,        498, 0 },
    { "pass_mot",                  VAR_UINT8,  &cfg.pass_mot,                    0,         10, 0 },
    { "rc_min",                    VAR_UINT16, &cfg.rc_min,                      0,       2000, 0 },
    { "rc_max",                    VAR_UINT16, &cfg.rc_max,                      0,       2000, 0 },
    { "rc_rllrm",                  VAR_UINT8,  &cfg.rc_rllrm,                    0,          1, 0 },
    { "rc_killt",                  VAR_UINT16, &cfg.rc_killt,                    0,      10000, 1 },
    { "fs_delay",                  VAR_UINT8,  &cfg.fs_delay,                    0,         40, 1 },
    { "fs_ofdel",                  VAR_UINT8,  &cfg.fs_ofdel,                    0,        200, 1 },
    { "fs_rcthr",                  VAR_UINT16, &cfg.fs_rcthr,                 1000,       2000, 1 },
    { "fs_ddplt",                  VAR_UINT8,  &cfg.fs_ddplt,                    0,        250, 1 },
    { "fs_jstph",                  VAR_UINT8,  &cfg.fs_jstph,                    0,          1, 1 },
    { "fs_nosnr",                  VAR_UINT8,  &cfg.fs_nosnr,                    0,          1, 1 },
    { "serial_baudrate",           VAR_UINT32, &cfg.serial_baudrate,          1200,     115200, 0 },
    { "tele_prot",                 VAR_UINT8,  &cfg.tele_prot,                   0,          3, 1 },
    { "spektrum_hires",            VAR_UINT8,  &cfg.spektrum_hires,              0,          1, 0 },
    { "vbatscale",                 VAR_UINT8,  &cfg.vbatscale,                  10,        200, 0 },
    { "vbatmaxcellvoltage",        VAR_UINT8,  &cfg.vbatmaxcellvoltage,         10,         50, 0 },
    { "vbatmincellvoltage",        VAR_UINT8,  &cfg.vbatmincellvoltage,         10,         50, 0 },
    { "power_adc_channel",         VAR_UINT8,  &cfg.power_adc_channel,           0,          9, 0 },
    { "tri_ydir",                  VAR_INT8,   &cfg.tri_ydir,                   -1,          1, 0 },
    { "tri_ymid",                  VAR_UINT16, &cfg.tri_ymid,                    0,       2000, 1 },
    { "tri_ymin",                  VAR_UINT16, &cfg.tri_ymin,                    0,       2000, 1 },
    { "tri_ymax",                  VAR_UINT16, &cfg.tri_ymax,                    0,       2000, 1 },
    { "tri_ydel",                  VAR_UINT16, &cfg.tri_ydel,                    0,       1000, 1 },    
    { "wing_left_min",             VAR_UINT16, &cfg.wing_left_min,               0,       2000, 0 },
    { "wing_left_mid",             VAR_UINT16, &cfg.wing_left_mid,               0,       2000, 0 },
    { "wing_left_max",             VAR_UINT16, &cfg.wing_left_max,               0,       2000, 0 },
    { "wing_right_min",            VAR_UINT16, &cfg.wing_right_min,              0,       2000, 0 },
    { "wing_right_mid",            VAR_UINT16, &cfg.wing_right_mid,              0,       2000, 0 },
    { "wing_right_max",            VAR_UINT16, &cfg.wing_right_max,              0,       2000, 0 },
    { "pitch_direction_l",         VAR_INT8,   &cfg.pitch_direction_l,          -1,          1, 0 },
    { "pitch_direction_r",         VAR_INT8,   &cfg.pitch_direction_r,          -1,          1, 0 },
    { "roll_direction_l",          VAR_INT8,   &cfg.roll_direction_l,           -1,          1, 0 },
    { "roll_direction_r",          VAR_INT8,   &cfg.roll_direction_r,           -1,          1, 0 },
    { "gbl_flg",                   VAR_UINT8,  &cfg.gbl_flg,                     0,        255, 0 },
    { "gbl_pgn",                   VAR_INT8,   &cfg.gbl_pgn,                  -100,        100, 0 },
    { "gbl_rgn",                   VAR_INT8,   &cfg.gbl_rgn,                  -100,        100, 0 },
    { "gbl_pmn",                   VAR_UINT16, &cfg.gbl_pmn,                   100,       3000, 0 },
    { "gbl_pmx",                   VAR_UINT16, &cfg.gbl_pmx,                   100,       3000, 0 },
    { "gbl_pmd",                   VAR_UINT16, &cfg.gbl_pmd,                   100,       3000, 0 },
    { "gbl_rmn",                   VAR_UINT16, &cfg.gbl_rmn,                   100,       3000, 0 },
    { "gbl_rmx",                   VAR_UINT16, &cfg.gbl_rmx,                   100,       3000, 0 },
    { "gbl_rmd",                   VAR_UINT16, &cfg.gbl_rmd,                   100,       3000, 0 },
    { "al_barolr",                 VAR_UINT8,  &cfg.al_barolr,                  10,        200, 1 },
    { "al_snrlr",                  VAR_UINT8,  &cfg.al_snrlr,                   10,        200, 1 },
    { "al_debounce",               VAR_UINT8,  &cfg.al_debounce,                 0,         20, 1 },
    { "al_tobaro",                 VAR_UINT16, &cfg.al_tobaro,                 100,       5000, 1 },
    { "al_tosnr",                  VAR_UINT16, &cfg.al_tosnr,                  100,       5000, 1 },
    { "as_lnchr",                  VAR_UINT8,  &cfg.as_lnchr,                   50,        250, 1 },
    { "as_clmbr",                  VAR_UINT8,  &cfg.as_clmbr,                   50,        250, 1 },
    { "as_trgt",                   VAR_UINT8,  &cfg.as_trgt,                     0,         15, 1 },
    { "as_stdev",                  VAR_UINT8,  &cfg.as_stdev,                    5,         20, 1 },
    { "align_gyro_x",              VAR_INT8,   &cfg.align[ALIGN_GYRO][0],       -3,          3, 0 },
    { "align_gyro_y",              VAR_INT8,   &cfg.align[ALIGN_GYRO][1],       -3,          3, 0 },
    { "align_gyro_z",              VAR_INT8,   &cfg.align[ALIGN_GYRO][2],       -3,          3, 0 },
    { "align_acc_x",               VAR_INT8,   &cfg.align[ALIGN_ACCEL][0],      -3,          3, 0 },
    { "align_acc_y",               VAR_INT8,   &cfg.align[ALIGN_ACCEL][1],      -3,          3, 0 },
    { "align_acc_z",               VAR_INT8,   &cfg.align[ALIGN_ACCEL][2],      -3,          3, 0 },
    { "align_mag_x",               VAR_INT8,   &cfg.align[ALIGN_MAG][0],        -3,          3, 0 },
    { "align_mag_y",               VAR_INT8,   &cfg.align[ALIGN_MAG][1],        -3,          3, 0 },
    { "align_mag_z",               VAR_INT8,   &cfg.align[ALIGN_MAG][2],        -3,          3, 0 },
    { "acc_hdw",                   VAR_UINT8,  &cfg.acc_hdw,                     0,          3, 0 },
    { "acc_lpf",                   VAR_UINT8,  &cfg.acc_lpf,                     1,        250, 1 },
    { "acc_ilpf",                  VAR_UINT8,  &cfg.acc_ilpf,                    1,        250, 1 },
    { "acc_trim_pitch",            VAR_INT16,  &cfg.angleTrim[PITCH],         -300,        300, 1 },
    { "acc_trim_roll",             VAR_INT16,  &cfg.angleTrim[ROLL],          -300,        300, 1 },
    { "gy_lpf",                    VAR_UINT16, &cfg.gy_lpf,                      0,        256, 0 },
    { "gy_cmpf",                   VAR_UINT16, &cfg.gy_cmpf,                    10,       2000, 1 },
    { "gy_cmpfm",                  VAR_UINT16, &cfg.gy_cmpfm,                   10,       2000, 1 },
    { "gy_smrll",                  VAR_UINT8,  &cfg.gy_smrll,                    0,        200, 1 },
    { "gy_smptc",                  VAR_UINT8,  &cfg.gy_smptc,                    0,        200, 1 },
    { "gy_smyw",                   VAR_UINT8,  &cfg.gy_smyw,                     0,        200, 1 },
    { "gy_stdev",                  VAR_UINT8,  &cfg.gy_stdev,                    5,        100, 0 },
    { "accz_vcf",                  VAR_FLOAT,  &cfg.accz_vcf,                    0,          1, 1 },
    { "accz_acf",                  VAR_FLOAT,  &cfg.accz_acf,                    0,          1, 1 },
    { "bar_lag",                   VAR_FLOAT,  &cfg.bar_lag,                     0,         10, 1 },
    { "bar_dscl",                  VAR_FLOAT,  &cfg.bar_dscl,                    0,          1, 1 },
    { "bar_dbg",                   VAR_UINT8,  &cfg.bar_dbg,                     0,          1, 0 },
    { "mag_dec",                   VAR_INT16,  &cfg.mag_dec,                -18000,      18000, 1 },
    { "mag_time",                  VAR_UINT8,  &cfg.mag_time,                    1,          6, 1 },
    { "mag_gain",                  VAR_UINT8,  &cfg.mag_gain,                    0,          1, 1 },
    { "gps_baudrate",              VAR_UINT32, &cfg.gps_baudrate,             1200,     115200, 0 },
    { "gps_type",                  VAR_UINT8,  &cfg.gps_type,                    0,          9, 0 },
    { "gps_ins_vel",               VAR_FLOAT,  &cfg.gps_ins_vel,                 0,          1, 1 },
    { "gps_ins_mdl",               VAR_UINT8,  &cfg.gps_ins_mdl,                 1,          2, 1 },
    { "gps_lag",                   VAR_UINT16, &cfg.gps_lag,                     0,      10000, 1 },
    { "gps_phase",                 VAR_INT8,   &cfg.gps_phase,                 -30,         30, 1 },
    { "gps_ph_minsat",             VAR_UINT8,  &cfg.gps_ph_minsat,               5,         10, 1 },
    { "gps_ph_settlespeed",        VAR_UINT8,  &cfg.gps_ph_settlespeed,          1,        200, 1 },
    { "gps_maxangle",              VAR_UINT8,  &cfg.gps_maxangle,               10,         45, 1 },
    { "gps_ph_brakemaxangle",      VAR_UINT8,  &cfg.gps_ph_brakemaxangle,        1,         45, 1 },
    { "gps_ph_minbrakepercent",    VAR_UINT8,  &cfg.gps_ph_minbrakepercent,      1,         99, 1 },
    { "gps_ph_brkacc",             VAR_UINT16, &cfg.gps_ph_brkacc,               1,        500, 1 },
    { "gps_ph_abstub",             VAR_UINT16, &cfg.gps_ph_abstub,               0,       1000, 1 },
    { "gps_wp_radius",             VAR_UINT16, &cfg.gps_wp_radius,               0,       2000, 1 },
    { "rtl_mnh",                   VAR_UINT8,  &cfg.rtl_mnh,                     0,        200, 1 },
    { "rtl_cr",                    VAR_UINT8,  &cfg.rtl_cr,                     10,        200, 1 },
    { "rtl_mnd",                   VAR_UINT8,  &cfg.rtl_mnd,                     0,         50, 1 },
    { "gps_rtl_flyaway",           VAR_UINT8,  &cfg.gps_rtl_flyaway,             0,        100, 1 },
    { "gps_yaw",                   VAR_UINT8,  &cfg.gps_yaw,                    20,        150, 1 },
    { "nav_rtl_lastturn",          VAR_UINT8,  &cfg.nav_rtl_lastturn,            0,          1, 1 },
    { "nav_speed_min",             VAR_UINT8,  &cfg.nav_speed_min,              10,        200, 1 },
    { "nav_speed_max",             VAR_UINT16, &cfg.nav_speed_max,              50,       2000, 1 },
    { "nav_approachdiv",           VAR_UINT8,  &cfg.nav_approachdiv,             2,         10, 1 },
    { "nav_tiltcomp",              VAR_UINT8,  &cfg.nav_tiltcomp,                0,        100, 1 },
    { "nav_ctrkgain",              VAR_FLOAT,  &cfg.nav_ctrkgain,                0,         10, 1 },
    { "nav_slew_rate",             VAR_UINT8,  &cfg.nav_slew_rate,               0,        200, 1 },
    { "nav_controls_heading",      VAR_UINT8,  &cfg.nav_controls_heading,        0,          1, 1 },
    { "nav_tail_first",            VAR_UINT8,  &cfg.nav_tail_first,              0,          1, 1 },
    { "floppy_mode",               VAR_UINT8,  &cfg.floppy_mode,                 0,          1, 1 },
    { "stat_clear",                VAR_UINT8,  &cfg.stat_clear,                  0,          1, 1 },    
    { "gps_pos_p",                 VAR_UINT8,  &cfg.P8[PIDPOS],                  0,        200, 1 },
    { "gps_pos_i",                 VAR_UINT8,  &cfg.I8[PIDPOS],                  0,        200, 0 },
    { "gps_pos_d",                 VAR_UINT8,  &cfg.D8[PIDPOS],                  0,        200, 0 },
    { "gps_posr_p",                VAR_UINT8,  &cfg.P8[PIDPOSR],                 0,        200, 1 },
    { "gps_posr_i",                VAR_UINT8,  &cfg.I8[PIDPOSR],                 0,        200, 1 },
    { "gps_posr_d",                VAR_UINT8,  &cfg.D8[PIDPOSR],                 0,        200, 1 },
    { "gps_nav_p",                 VAR_UINT8,  &cfg.P8[PIDNAVR],                 0,        200, 1 },
    { "gps_nav_i",                 VAR_UINT8,  &cfg.I8[PIDNAVR],                 0,        200, 1 },
    { "gps_nav_d",                 VAR_UINT8,  &cfg.D8[PIDNAVR],                 0,        200, 1 },
    { "looptime",                  VAR_UINT16, &cfg.looptime,                    0,       9000, 1 },
    { "mainpidctrl",               VAR_UINT8,  &cfg.mainpidctrl,                 0,          1, 1 },
    { "mainpt1cut",                VAR_UINT8,  &cfg.mainpt1cut,                  0,         50, 1 },
    { "newpidimax",                VAR_UINT16, &cfg.newpidimax,                 10,      65000, 1 },
    { "gpspt1cut",                 VAR_UINT8,  &cfg.gpspt1cut,                   1,         50, 1 },    
    { "p_pitch",                   VAR_UINT8,  &cfg.P8[PITCH],                   0,        200, 1 },
    { "i_pitch",                   VAR_UINT8,  &cfg.I8[PITCH],                   0,        200, 1 },
    { "d_pitch",                   VAR_UINT8,  &cfg.D8[PITCH],                   0,        200, 1 },
    { "p_roll",                    VAR_UINT8,  &cfg.P8[ROLL],                    0,        200, 1 },
    { "i_roll",                    VAR_UINT8,  &cfg.I8[ROLL],                    0,        200, 1 },
    { "d_roll",                    VAR_UINT8,  &cfg.D8[ROLL],                    0,        200, 1 },
    { "p_yaw",                     VAR_UINT8,  &cfg.P8[YAW],                     0,        200, 1 },
    { "i_yaw",                     VAR_UINT8,  &cfg.I8[YAW],                     0,        200, 1 },
    { "d_yaw",                     VAR_UINT8,  &cfg.D8[YAW],                     0,        200, 1 },
    { "p_alt",                     VAR_UINT8,  &cfg.P8[PIDALT],                  0,        200, 1 },
    { "i_alt",                     VAR_UINT8,  &cfg.I8[PIDALT],                  0,        200, 1 },
    { "d_alt",                     VAR_UINT8,  &cfg.D8[PIDALT],                  0,        200, 1 },
    { "p_level",                   VAR_UINT8,  &cfg.P8[PIDLEVEL],                0,        200, 1 },
    { "i_level",                   VAR_UINT8,  &cfg.I8[PIDLEVEL],                0,        200, 1 },
    { "d_level",                   VAR_UINT8,  &cfg.D8[PIDLEVEL],                0,        200, 1 },
    { "snr_type",                  VAR_UINT8,  &cfg.snr_type,                    0,          4, 0 },
    { "snr_min",                   VAR_UINT8,  &cfg.snr_min,                    10,        200, 1 },
    { "snr_max",                   VAR_UINT16, &cfg.snr_max,                    50,        700, 1 },
    { "snr_dbg",                   VAR_UINT8,  &cfg.snr_dbg,                     0,          1, 0 },
    { "snr_tilt",                  VAR_UINT8,  &cfg.snr_tilt,                   10,         50, 1 },
    { "snr_cf",                    VAR_FLOAT,  &cfg.snr_cf,                      0,          1, 1 },
    { "snr_diff",                  VAR_UINT8,  &cfg.snr_diff,                    0,        200, 1 },
    { "snr_land",                  VAR_UINT8,  &cfg.snr_land,                    0,          1, 1 },    
    { "LED_invert",                VAR_UINT8,  &cfg.LED_invert,                  0,          1, 0 },
    { "LED_Type",                  VAR_UINT8,  &cfg.LED_Type,                    0,          3, 0 },
    { "LED_pinout",                VAR_UINT8,  &cfg.LED_Pinout,                  0,          1, 0 },
    { "LED_ControlChannel",        VAR_UINT8,  &cfg.LED_ControlChannel,          1,         12, 0 }, // Aux Channel to controll the LED Pattern
    { "LED_ARMED",                 VAR_UINT8,  &cfg.LED_Armed,                   0,          1, 1 }, // 0 = Show LED only if armed, 1 = always show LED
    { "LED_Toggle_Delay1",         VAR_UINT8,  &cfg.LED_Toggle_Delay1,           0,        255, 0 },
    { "LED_Toggle_Delay2",         VAR_UINT8,  &cfg.LED_Toggle_Delay2,           0,        255, 0 },
    { "LED_Toggle_Delay3",         VAR_UINT8,  &cfg.LED_Toggle_Delay3,           0,        255, 0 },
    { "LED_Pattern1",              VAR_UINT32, &cfg.LED_Pattern1,                0, 0x7FFFFFFF, 0 }, // Pattern for Switch position 1
    { "LED_Pattern2",              VAR_UINT32, &cfg.LED_Pattern2,                0, 0x7FFFFFFF, 0 }, // Pattern for Switch position 2
    { "LED_Pattern3",              VAR_UINT32, &cfg.LED_Pattern3,                0, 0x7FFFFFFF, 0 }, // Pattern for Switch position 3};
};

#define VALUE_COUNT (sizeof(valueTable) / sizeof(valueTable[0]))

#ifndef HAVE_ITOA_FUNCTION

/*
** The following two functions together make up an itoa()
** implementation. Function i2a() is a 'private' function
** called by the public itoa() function.
**
** itoa() takes three arguments:
**        1) the integer to be converted,
**        2) a pointer to a character conversion buffer,
**        3) the radix for the conversion
**           which can range between 2 and 36 inclusive
**           range errors on the radix default it to base10
** Code from http://groups.google.com/group/comp.lang.c/msg/66552ef8b04fe1ab?pli=1
*/

static char *i2a(unsigned i, char *a, unsigned r)
{
    if (i / r > 0)
        a = i2a(i / r, a, r);
    *a = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"[i % r];
    return a + 1;
}

char *itoa(int i, char *a, int r)
{
    if ((r < 2) || (r > 36))
        r = 10;
    if (i < 0)
    {
        *a = '-';
        *i2a(-(unsigned)i, a + 1, r) = 0;
    }
    else
        *i2a(i, a, r) = 0;
    return a;
}

#endif

////////////////////////////////////////////////////////////////////////////////
// String to Float Conversion
///////////////////////////////////////////////////////////////////////////////
// Simple and fast atof (ascii to float) function.
//
// - Executes about 5x faster than standard MSCRT library atof().
// - An attractive alternative if the number of calls is in the millions.
// - Assumes input is a proper integer, fraction, or scientific format.
// - Matches library atof() to 15 digits (except at extreme exponents).
// - Follows atof() precedent of essentially no error checking.
//
// 09-May-2009 Tom Van Baak (tvb) www.LeapSecond.com
//
#define white_space(c) ((c) == ' ' || (c) == '\t')
#define valid_digit(c) ((c) >= '0' && (c) <= '9')
static float _atof(const char *p)
{
    int frac = 0;
    double sign, value, scale;

    // Skip leading white space, if any.
    while (white_space(*p) )
    {
        p += 1;
    }

    // Get sign, if any.
    sign = 1.0;
    if (*p == '-')
    {
        sign = -1.0;
        p += 1;

    }
    else if (*p == '+')
    {
        p += 1;
    }

    // Get digits before decimal point or exponent, if any.
    value = 0.0;
    while (valid_digit(*p))
    {
        value = value * 10.0 + (*p - '0');
        p += 1;
    }

    // Get digits after decimal point, if any.
    if (*p == '.')
    {
        double pow10 = 10.0;
        p += 1;

        while (valid_digit(*p))
        {
            value += (*p - '0') / pow10;
            pow10 *= 10.0;
            p += 1;
        }
    }

    // Handle exponent, if any.
    scale = 1.0;
    if ((*p == 'e') || (*p == 'E'))
    {
        unsigned int expon;
        p += 1;

        // Get sign of exponent, if any.
        frac = 0;
        if (*p == '-')
        {
            frac = 1;
            p += 1;

        }
        else if (*p == '+')
        {
            p += 1;
        }

        // Get digits of exponent, if any.
        expon = 0;
        while (valid_digit(*p))
        {
            expon = expon * 10 + (*p - '0');
            p += 1;
        }
        if (expon > 308) expon = 308;

        // Calculate scaling factor.
        while (expon >= 50)
        {
            scale *= 1E50;
            expon -= 50;
        }
        while (expon >=  8)
        {
            scale *= 1E8;
            expon -=  8;
        }
        while (expon >   0)
        {
            scale *= 10.0;
            expon -=  1;
        }
    }

    // Return signed and scaled floating point result.
    return sign * (frac ? (value / scale) : (value * scale));
}

///////////////////////////////////////////////////////////////////////////////
// FTOA
///////////////////////////////////////////////////////////////////////////////
static char *ftoa(float x, char *floatString)
{
    int32_t value;
    char intString1[12];
    char intString2[12] = { 0, };
    char *decimalPoint = ".";
    uint8_t dpLocation;

    if (x > 0) x += 0.0005f;
    else x -= 0.0005f;
    value = (int32_t) (x * 1000.0f);    // Convert float * 1000 to an integer

    itoa(abs(value), intString1, 10);   // Create string from abs of integer value

    if (value >= 0) intString2[0] = ' ';// Positive number, add a pad space
    else intString2[0] = '-';           // Negative number, add a negative sign

    if (strlen(intString1) == 1)
    {
        intString2[1] = '0';
        intString2[2] = '0';
        intString2[3] = '0';
        strcat(intString2, intString1);
    }
    else if (strlen(intString1) == 2)
    {
        intString2[1] = '0';
        intString2[2] = '0';
        strcat(intString2, intString1);
    }
    else if (strlen(intString1) == 3)
    {
        intString2[1] = '0';
        strcat(intString2, intString1);
    }
    else
    {
        strcat(intString2, intString1);
    }

    dpLocation = strlen(intString2) - 3;

    strncpy(floatString, intString2, dpLocation);
    floatString[dpLocation] = '\0';
    strcat(floatString, decimalPoint);
    strcat(floatString, intString2 + dpLocation);
    return floatString;
}

static void cliPrompt(void)
{
    uartPrint("\r\n# ");
}

static int cliCompare(const void *a, const void *b)
{
    const clicmd_t *ca = a, *cb = b;
    return strncasecmp(ca->name, cb->name, strlen(cb->name));
}

#ifdef debugmode
static void printxyzcalval(float *off, float *dev, bool onlyoffset)
{
    uint8_t i;
    char    k = 'X';
    for (i = 0; i < 3; i++)
    {
        if(onlyoffset)printf("\r\n%c Offset: %d", k, (int32_t)off[i]);
        else printf("\r\n%c Offset: %d, StdDev: %d", k, (int32_t)off[i], (int32_t)dev[i]);
        k++;
    }
}

static void cliDebug(char *cmdline)
{
    float   tmp[3];
    uint8_t i;
    char    k = 'X';
  
    printf("\r\nGYRO Runtime StdDev*100");
    for (i = 0; i < 3; i++) tmp[i] = gyrostddev[i] * 100;
    printxyzcalval(gyroZero, tmp, false);

    if (sensors(SENSOR_ACC))
    {
        printf("\r\n\r\nACC StdDev*100");
        for (i = 0; i < 3; i++) tmp[i] = cfg.accstddev[i] * 100;
        printxyzcalval(cfg.accZero, tmp, false);
        printf("\r\n1G Sensorval: %d", (int32_t)cfg.sens_1G);
    }

    if (sensors(SENSOR_MAG))
    {
        printf("\r\n\r\nMAG ");
        if (cfg.mag_calibrated) printf("calibrated"); else printf("not calibrated");
        printxyzcalval(cfg.magZero, 0, true);
        printf("\r\nRuntime\r\n");
        for (i = 0; i < 3; i++)
        {
            printf("%c Gain*1000: %d\r\n", k, (int32_t)(magCal[i] * 1000));      
            k++;
        }
        printf("Gain ");
        if (maggainok) printf("OK\r\n"); else printf("NOT OK\r\n");
    }

#ifdef barotempdebug
    if (sensors(SENSOR_BARO))
    {
        printf("\r\n\r\nBARO\r\n");
        printf("Measured Temp: %d\r\n",(int32_t)BaroActualTemp);      
    }
#endif
}
#endif

static void PrintBox(uint8_t number, bool fillup)      // Prints Out Boxname, left aligned and 8 chars. Cropping or filling with blank may occur
{
#define MaxCharInline 8
    uint8_t i = 0, k = 0;
    bool DoBlank = false;

    if (number >= CHECKBOXITEMS) return;
    while (k != number)
    {
        if(boxnames[i] == ';') k++;
        i++;
    }
    k = i;
    for (i = 0; i < MaxCharInline; i++)
    {
        if (boxnames[k + i] == ';')
        {
            if (fillup) DoBlank = true;
             else return;
        }
        if (DoBlank) printf(" ");
         else printf("%c", boxnames[k + i]);          
    }
}

static void cliAuxset(char *cmdline)
{
    uint8_t  i, k, ItemID, AuxChNr;
    uint32_t val = 1;
    bool     remove = false;
    char     *ptr = cmdline;                       // ptr = cmdline;
    char     buf[4];
    uint8_t  len = strlen(cmdline);

    if (!len || len < 5)
    {
        printf("\r\nSet: auxset ID aux state(H/M/L)\r\n");
        printf("Remove: auxset -ID etc.\r\n");
        printf("Ex: auxset 1 4 h Sets Box 1 to Aux4 High\r\n\r\n");
        printf("ID|AUXCHAN :");
        for (i = 0; i < cfg.rc_auxch; i++) printf(" %02u  ", i + 1);
        for (i = 0; i < CHECKBOXITEMS; i++)        // print out aux channel settings
        {
            printf("\r\n%02u|", i);
            PrintBox(i, true);
            printf(":");
            for (k = 0; k < cfg.rc_auxch; k++)
            {
                strcpy(buf,"--- ");
                val = cfg.activate[i];
                val = val >> (k * 3);
                if (val & 1) buf[0] = 'L';
                if (val & 2) buf[1] = 'M';
                if (val & 4) buf[2] = 'H';
                printf(" %s", buf);
            }
        }
        printf("\r\n");
    }
    else
    {
        if (ptr[0] == '-')
        {
            remove = true;
            ptr++;
        }
        ItemID = atoi(ptr);
        if (ItemID < CHECKBOXITEMS)
        {
            ptr = strchr(ptr, ' ');
            AuxChNr = atoi(ptr);
            if (AuxChNr > cfg.rc_auxch || !AuxChNr)
            {
                cliErrorMessage();
                return;
            }
            AuxChNr--;
            ptr = strchr(ptr + 1, ' ');
            i   = AuxChNr * 3;
            switch(ptr[1])
            {
            case 'L':
            case 'l':
                strcpy(buf,"LOW ");
                val <<= i;
                break;
            case 'M':
            case 'm':
                strcpy(buf,"MED ");
                val <<= (i + 1);
                break;
            case 'H':
            case 'h':
                strcpy(buf,"HIGH");
                val <<= (i + 2);
                break;
	          default:
                cliErrorMessage();
                return;
            }
            cfg.activate[ItemID] |= val;           // Set it here in any case, so we can eor it away if needed
            if(remove)
            {
                cfg.activate[ItemID] ^= val;
                printf("Removing ");
            }
            else
            {
                printf("Setting ");
            }
            PrintBox(ItemID, false);
            printf(" Aux %02u %s", AuxChNr + 1, buf);
        }
        else
        {
            cliErrorMessage();
            return;
        }
    }
}

static void cliCMix(char *cmdline)
{
    int i, check = 0;
    int num_motors = 0;
    uint8_t len;
    char buf[16];
    float mixsum[3];
    char *ptr;

    len = strlen(cmdline);

    if (!len)
    {
        uartPrint("Custom mixer: \r\nMotor\tThr\tRoll\tPitch\tYaw\r\n");
        for (i = 0; i < MAX_MOTORS; i++)
        {
            if (cfg.customMixer[i].throttle == 0.0f)
                break;
            num_motors++;
            printf("#%d:\t", i + 1);
            printf("%s\t", ftoa(cfg.customMixer[i].throttle, buf));
            printf("%s\t", ftoa(cfg.customMixer[i].roll, buf));
            printf("%s\t", ftoa(cfg.customMixer[i].pitch, buf));
            printf("%s\r\n", ftoa(cfg.customMixer[i].yaw, buf));
        }
        for (i = 0; i < 3; i++)                                           // Fix by meister
            mixsum[i] = 0.0f;
        for (i = 0; i < num_motors; i++)
        {
            mixsum[0] += cfg.customMixer[i].roll;
            mixsum[1] += cfg.customMixer[i].pitch;
            mixsum[2] += cfg.customMixer[i].yaw;
        }
        uartPrint("Sanity check:\t");
        for (i = 0; i < 3; i++)
            uartPrint(fabs(mixsum[i]) > 0.01f ? "NG\t" : "OK\t");
        uartPrint("\r\n");
        return;
    }
    else if (strncasecmp(cmdline, "load", 4) == 0)
    {
        ptr = strchr(cmdline, ' ');
        if (ptr)
        {
            len = strlen(++ptr);
            for (i = 0; ; i++)
            {
                if (mixerNames[i] == NULL)
                {
                    cliErrorMessage();                   // uartPrint("Invalid mixer type...\r\n");
                    break;
                }
                if (strncasecmp(ptr, mixerNames[i], len) == 0)
                {
                    mixerLoadMix(i);
                    printf("Loaded %s mix...\r\n", mixerNames[i]);
                    cliCMix("");
                    break;
                }
            }
        }
    }
    else
    {
        ptr = cmdline;
        i = atoi(ptr); // get motor number
        if (--i < MAX_MOTORS)
        {
            ptr = strchr(ptr, ' ');
            if (ptr)
            {
                cfg.customMixer[i].throttle = _atof(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr)
            {
                cfg.customMixer[i].roll = _atof(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr)
            {
                cfg.customMixer[i].pitch = _atof(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr)
            {
                cfg.customMixer[i].yaw = _atof(++ptr);
                check++;
            }
            if (check != 4) uartPrint("Invalid arguments, needs idx thr roll pitch yaw\r\n");
            else cliCMix("");
        }
        else printf("Motor nr not in range 1 - %d\r\n", MAX_MOTORS);
    }
}

static void cliDefault(char *cmdline)
{
    uartPrint("Resetting to defaults...\r\n");
    checkFirstTime(true);
    uartPrint("Rebooting...");
    delay(10);
    systemReset(false);
}

static void cliDump(char *cmdline)
{
    int i;                                         //, val = 0;
    char buf[16];
    float thr, roll, pitch, yaw;
    const clivalue_t *setval;

    printf(";Actual Config:\r\n");
    printf(";Firmware: %s\r\n", FIRMWARE);
    cliAuxset("");
    printf("mixer %s\r\n", mixerNames[cfg.mixerConfiguration - 1]); // print out current motor mix
    if (cfg.customMixer[0].throttle != 0.0f)       // print custom mix if exists
    {
        for (i = 0; i < MAX_MOTORS; i++)
        {
            if (cfg.customMixer[i].throttle == 0.0f) break;
            thr   = cfg.customMixer[i].throttle;
            roll  = cfg.customMixer[i].roll;
            pitch = cfg.customMixer[i].pitch;
            yaw = cfg.customMixer[i].yaw;
            printf("cmix %d", i + 1);
            if (thr < 0) printf(" ");
            printf("%s", ftoa(thr, buf));
            if (roll < 0) printf(" ");
            printf("%s", ftoa(roll, buf));
            if (pitch < 0) printf(" ");
            printf("%s", ftoa(pitch, buf));
            if (yaw < 0) printf(" ");
            printf("%s\r\n", ftoa(yaw, buf));
        }
        printf("cmix %d 0 0 0 0\r\n", i + 1);
    }
    cliFeature("");
    for (i = 0; i < 8; i++) buf[cfg.rcmap[i]] = rcChannelLetters[i]; // print RC MAPPING
    buf[i] = '\0';
    printf("map %s\r\n", buf);
    for (i = 0; i < VALUE_COUNT; i++)                    // print settings
    {
        setval = &valueTable[i];
        printf("set %s = ", valueTable[i].name);
        cliPrintVar(setval, 0);
        uartPrint("\r\n");
    }
}

static void cliFeature(char *cmdline)
{
    uint32_t i;
    uint32_t len;
    uint32_t mask;

    len = strlen(cmdline);
    mask = featureMask();

    if (!len)
    {
        uartPrint("Enabled features: ");
        for (i = 0; ; i++)
        {
            if (featureNames[i] == NULL) break;
            if (mask & (1 << i)) printf("%s ", featureNames[i]);
        }
        uartPrint("\r\n");
    }
    else if (strncasecmp(cmdline, "list", len) == 0)
    {
        uartPrint("Available features: \r\n");
        for (i = 0; ; i++)
        {
            if (featureNames[i] == NULL) break;
            printf("%s \r\n", featureNames[i]);
        }
        uartPrint("\r\n");
        return;
    }
    else
    {
        bool remove = false;
        bool fpass  = feature(FEATURE_PASS);
        if (cmdline[0] == '-')
        {
            remove = true;            // remove feature
            cmdline++;                // skip over -
            len--;
        }

        for (i = 0; ; i++)
        {
            if (featureNames[i] == NULL)
            {
                uartPrint("Invalid feature name\r\n");
                break;
            }
            if (strncasecmp(cmdline, featureNames[i], len) == 0)
            {
                if (remove)
                {
                    featureClear(1 << i);
                    uartPrint("Disabled ");
                }
                else
                {
                    featureSet(1 << i);
                    uartPrint("Enabled ");
                }
                if (fpass != feature(FEATURE_PASS)) cfg.pass_mot = 0; // Reset to all motors if feature pass was changed
                printf("%s\r\n", featureNames[i]);
                break;
            }
        }
    }
}

static void cliHelp(char *cmdline)
{
    uint32_t i = 0;
    uartPrint("Available commands:\r\n\r\n");
    for (i = 0; i < CMD_COUNT; i++) printf("%s\t %s\r\n", cmdTable[i].name, cmdTable[i].param);
}

static void cliMap(char *cmdline)
{
    uint32_t len;
    uint32_t i;
    char out[9];
    len = strlen(cmdline);
    if (len == 8)
    {
        // uppercase it
        for (i = 0; i < 8; i++) cmdline[i] = toupper((unsigned char)cmdline[i]); // toupper(cmdline[i]);
        for (i = 0; i < 8; i++)
        {
            if (strchr(rcChannelLetters, cmdline[i]) && !strchr(cmdline + i + 1, cmdline[i]))
                continue;
            uartPrint("Must be any order of AETR1234\r\n");
            return;
        }
        parseRcChannels(cmdline);
    }
    uartPrint("Current assignment: ");
    for (i = 0; i < 8; i++) out[cfg.rcmap[i]] = rcChannelLetters[i];
    out[i] = '\0';
    printf("%s\r\n", out);
}

static void cliMixer(char *cmdline)
{
    uint8_t i;
    uint8_t len;

    len = strlen(cmdline);

    if (!len)
    {
        printf("Current mixer: %s\r\n", mixerNames[cfg.mixerConfiguration - 1]);
        return;
    }
    else if (strncasecmp(cmdline, "list", len) == 0)
    {
        uartPrint("Available mixers: ");
        for (i = 0; ; i++)
        {
            if (mixerNames[i] == NULL) break;
            printf("%s ", mixerNames[i]);
        }
        uartPrint("\r\n");
        return;
    }

    for (i = 0; ; i++)
    {
        if (mixerNames[i] == NULL)
        {
            uartPrint("Invalid mixer type...\r\n");
            break;
        }
        if (strncasecmp(cmdline, mixerNames[i], len) == 0)
        {
            cfg.mixerConfiguration = i + 1;
            printf("Mixer set to %s\r\n", mixerNames[i]);
            break;
        }
    }
}

static void cliExit(char *cmdline)
{
    uartPrint("\r\nLeaving CLI mode without saving\r\n");
    memset(cliBuffer, 0, sizeof(cliBuffer));
    bufferIndex = 0;
    cliMode = 0;
    uartPrint("\r\nRebooting...");
    delay(10);
    systemReset(false);                                 // Just Reset without saving makes more sense
}

void cliSave(char *cmdline)
{
    uartPrint("Saving...");
    writeParams(0);
    uartPrint("\r\nRebooting...");
    delay(10);
    cliMode = 0;
    systemReset(false);
}

static void cliPrintVar(const clivalue_t *var, uint32_t full)
{
    int32_t value = 0;
    char buf[8];

    switch (var->type)
    {
    case VAR_UINT8:
        value = *(uint8_t *)var->ptr;
        break;

    case VAR_INT8:
        value = *(int8_t *)var->ptr;
        break;

    case VAR_UINT16:
        value = *(uint16_t *)var->ptr;
        break;

    case VAR_INT16:
        value = *(int16_t *)var->ptr;
        break;

    case VAR_UINT32:
        value = *(uint32_t *)var->ptr;
        break;

    case VAR_FLOAT:
        printf("%s", ftoa(*(float *)var->ptr, buf));
        if (full)
        {
            printf(" %s", ftoa((float)var->min, buf));
            printf(" %s", ftoa((float)var->max, buf));
        }
        return; // return from case for float only
    }
    printf("%d", value);
    if (full)
        printf(" %d %d", var->min, var->max);
}

static void cliSetVar(const clivalue_t *var, const int32_t value)
{
    switch (var->type)
    {
    case VAR_UINT8:
    case VAR_INT8:
        *(char *)var->ptr = (char)value;
        break;

    case VAR_UINT16:
    case VAR_INT16:
        *(short *)var->ptr = (short)value;
        break;

    case VAR_UINT32:
        *(int *)var->ptr = (int)value;
        break;

    case VAR_FLOAT:
        *(float *)var->ptr = *(float *)&value;
        break;
    }
}

static void cliSet(char *cmdline)
{
    uint32_t i;
    uint32_t len;
    const clivalue_t *val;
    char *eqptr = NULL;
    int32_t value = 0;
    float valuef = 0;

    len = strlen(cmdline);

    if (!len || (len == 1 && cmdline[0] == '*'))
    {
        uartPrint("Current settings: \r\n");
        for (i = 0; i < VALUE_COUNT; i++)
        {
            val = &valueTable[i];
            printf("%s = ", valueTable[i].name);
            cliPrintVar(val, len); // when len is 1 (when * is passed as argument), it will print min/max values as well, for gui
            uartPrint("\r\n");
        }
    }
    else if ((eqptr = strstr(cmdline, "=")))
    {
        // has equal, set var
        eqptr++;
        len--;
        value = atoi(eqptr);
        valuef = _atof(eqptr);
        for (i = 0; i < VALUE_COUNT; i++)
        {
            val = &valueTable[i];
            if (strncasecmp(cmdline, valueTable[i].name, strlen(valueTable[i].name)) == 0)
            {
                if (valuef >= valueTable[i].min && valuef <= valueTable[i].max)   // here we compare the float value since... it should work, RIGHT?
                {
                    cliSetVar(val, valueTable[i].type == VAR_FLOAT ? *(uint32_t *)&valuef : value); // this is a silly dirty hack. please fix me later.
                    printf("%s set to ", valueTable[i].name);
                    cliPrintVar(val, 0);
                }
                else
                {
                    cliErrorMessage();    // uartPrint("ERR: Value assignment out of range\r\n");
                }
                return;
            }
        }
        cliErrorMessage();  // uartPrint("ERR: Unknown variable name\r\n");
    }
}

static void cliStatus(char *cmdline)
{
    uint8_t  i, k;
    uint32_t mask;
    uint16_t tmpu16;
    printf("\r\nSystem Uptime: %d sec, Volt: %d * 0.1V (%dS battery)\r\n", currentTimeMS / 1000, vbat, batteryCellCount);
    mask = sensorsMask();
    printf("CPU %dMHz, detected sensors: ", (SystemCoreClock / 1000000));
    for (i = 0; ; i++)
    {
        if (sensorNames[i] == NULL) break;
        if (mask & (1 << i)) printf("%s ", sensorNames[i]);
    }
    if (sensors(SENSOR_ACC)) printf("ACC: %s", accNames[accHardware]);
    printf("\r\nCycle Time: %d, I2C Errors: %d\r\n\r\n", cycleTime, i2cGetErrorCounter());
    printf("Total : %d B\r\n", cfg.size);
    printf("Config: %d B\r\n", cfg.size - FDByteSize);
    printf("Logger: %d B, %d Datasets\r\n\r\n", FDByteSize, cfg.FDUsedDatasets);

    printf("Stats:\r\n");
    if (sensors(SENSOR_BARO) || sensors(SENSOR_GPS))
    {
        if (sensors(SENSOR_GPS))
        {
            printf("\r\nGPS:\r\n");
            tmpu16 = (uint16_t)((float)cfg.MAXGPSspeed * 0.036f);
            printf("Max Dist: %d m\r\n", cfg.GPS_MaxDistToHome );
            printf("Max Speed: %dcm/s = %dKm/h\r\n", cfg.MAXGPSspeed, tmpu16);
        }
        if (sensors(SENSOR_BARO))
        {
            printf("\r\nBaro:\r\n");
            printf("Max Alt AGL: %d m\r\n", cfg.MaxAltMeter);
            printf("Min Alt AGL: %d m\r\n", cfg.MinAltMeter);
        }
    }
    printf("\r\nMotor:\r\n");
    printf("Actual Range: %d - %d at %d Hz PWM.\r\n", cfg.esc_min, cfg.esc_max, cfg.esc_pwm);
    tmpu16 = (cfg.esc_max - cfg.esc_min) / 100;
    if(motorpercent[0])
    {
        k = min(NumberOfMotors, MAX_MONITORED_MOTORS);
        for (i = 0; i < k; i++) printf("Mot: %d Session Usage: %d%% Abs PWM: %d Rel to PWM range: %d%%\r\n", i + 1, motorpercent[i], motorabspwm[i],(motorabspwm[i] - cfg.esc_min) / tmpu16);
    } else printf("Nothing!\r\n");
}

static void cliVersion(char *cmdline)
{
    uartPrint(FIRMWARE);
}

void serialOSD(void)
{
#define LCDdelay 12
#define RcEndpoint 50
    static uint32_t  rctimer;
    static uint8_t   exit, input, lastinput, brake, brakeval, speeduptimer;
    static uint16_t  DatasetNr;
    const clivalue_t *setval;
    uint32_t timetmp;

    if (cliMode != 0) return;                                           // Don't do this if we are in cli mode
    LCDinit();
    printf(FIRMWAREFORLCD);                                             // Defined in mw.h
    LCDline2();
    printf("LCD Interface");
    delay(2000);
    LCDclear();
    exit = brake = brakeval = speeduptimer = 0;
    DatasetNr = 0;
    LCDline1();
    printf("%s", valueTable[DatasetNr].name);                           // Display first item anyway (even if lcd ==0) no need for special attention
    LCDline2();
    setval = &valueTable[DatasetNr];
    cliPrintVar(setval, 0);
    while (!exit)
    {
        timetmp = micros();
        if (spektrumFrameComplete() || graupnersumhFrameComplete()) computeRC(); // Generates no rcData yet, but rcDataSAVE
        if ((int32_t)(timetmp - rctimer) >= 0)                          // Start of 50Hz Loop
        {
            rctimer = timetmp + 20000;
            LED1_TOGGLE;
            LED0_TOGGLE;
            if (!feature(FEATURE_SPEKTRUM) && !feature(FEATURE_GRAUPNERSUMH)) computeRC();
            GetActualRCdataOutRCDataSave();                             // Now we have new rcData to deal and MESS with
            if (rcData[THROTTLE] < (cfg.rc_min + RcEndpoint) && rcData[PITCH] > (cfg.rc_max - RcEndpoint))
            {
                if (rcData[YAW] > (cfg.rc_max - RcEndpoint)) exit = 1;  // Quit don't save
                if (rcData[YAW] < (cfg.rc_min + RcEndpoint)) exit = 2;  // Quit and save
            }

            input = 0;
            
            if (!exit)
            {
                if (rcData[PITCH] < (cfg.rc_min + RcEndpoint)) input = 1;
                if (rcData[PITCH] > (cfg.rc_max - RcEndpoint)) input = 2;
                if (rcData[ROLL]  < (cfg.rc_min + RcEndpoint)) input = 3;
                if (rcData[ROLL]  > (cfg.rc_max - RcEndpoint)) input = 4;
            }

            if (lastinput == input)                                     // Adjust Inputspeed
            {
                speeduptimer++;
                if (speeduptimer >= 100)
                {
                    speeduptimer = 99;
                    brakeval = 8;
                }
                else brakeval = 17;
            }
            else
            {
                brakeval = 0;
                speeduptimer = 0;
            }
            lastinput = input;
            brake++;
            if (brake >= brakeval) brake = 0;
            else input = 0;

            switch (input)
            {
            case 0:
                break;
            case 1:                                                 // Down
                do                                                  // Search for next Dataset
                {
                    DatasetNr++;
                    if (DatasetNr == VALUE_COUNT) DatasetNr = 0;
                }
                while (!valueTable[DatasetNr].lcd);
                LCDclear();
                printf("%s", valueTable[DatasetNr].name);
                LCDline2();
                setval = &valueTable[DatasetNr];
                cliPrintVar(setval, 0);
                break;
            case 2:                                                 // UP
                do                                                  // Search for next Dataset
                {
                    if (!DatasetNr) DatasetNr = VALUE_COUNT;
                    DatasetNr--;
                }
                while (!valueTable[DatasetNr].lcd);
                LCDclear();
                printf("%s", valueTable[DatasetNr].name);
                LCDline2();
                setval = &valueTable[DatasetNr];
                cliPrintVar(setval, 0);
                break;
            case 3:                                                 // LEFT
                if (brakeval != 8) changeval(setval,-1);            // Substract within the limit
                else changeval(setval,-5);
                LCDline2();
                cliPrintVar(setval, 0);
                break;
            case 4:                                                 // RIGHT
                if (brakeval != 8) changeval(setval,1);             // Add within the limit
                else changeval(setval,5);
                LCDline2();
                cliPrintVar(setval, 0);
                break;
            }
        }                                                           // End of 50Hz Loop
    }
    delay(500);
    LCDclear();
    printf(" Exit & Reboot ");
    LCDline2();
    switch (exit)
    {
    case 1:
        printf(" NOT Saving");
        delay(1000);
        LCDoff();
        systemReset(false);
        break;
    case 2:
        printf(".!.!.Saving.!.!.");
        delay(1000);
        writeParams(0);
        LCDoff();
        systemReset(false);
        break;
    }
}

static void changeval(const clivalue_t *var, const int8_t adder)
{
    int32_t value, maximum, minimum;
    float   valuef;

    maximum = var->max;
    minimum = var->min;
    switch (var->type)
    {
    case VAR_UINT8:
    case VAR_INT8:
        value = *(char *)var->ptr;
        value = constrain(value + adder,minimum,maximum);
        *(char *)var->ptr = (char)value;
        break;

    case VAR_UINT16:
    case VAR_INT16:
        value = *(short *)var->ptr;
        value = constrain(value + adder,minimum,maximum);
        *(short *)var->ptr = (short)value;
        break;

    case VAR_UINT32:
        value = *(int *)var->ptr;
        value = constrain(value + adder,minimum,maximum);
        *(int *)var->ptr = (int)value;
        break;

    case VAR_FLOAT:
        *(float *)&valuef = *(float *)var->ptr;
        valuef = constrain(valuef + (float)adder/1000.0f,minimum,maximum);
        *(float *)var->ptr = *(float *)&valuef;
        break;
    }
}

void LCDinit(void)                                                      // changed Johannes
{
    if (!initI2cLCD(true))                                              // Will set i2cLCD
    {
        serialInit(9600);                                               // INIT LCD HERE
        LCDoff();
        uartWrite(0xFE);
        delay(LCDdelay);
        uartWrite(0x0C);                                                // Display ON
        delay(LCDdelay);
        uartWrite(0x7C);
        delay(LCDdelay);
        uartWrite(0x9D);                                                // 100% Brightness
        LCDclear();      
    }
}

void LCDoff(void)
{
    if (i2cLCD)                                                         // Johannes
        i2c_clear_OLED();
    else
    {
        delay(LCDdelay);
        uartWrite(0xFE);
        delay(LCDdelay);
        uartWrite(0x08);                                                // LCD Display OFF
        delay(LCDdelay);
    }
}

void LCDclear(void)                                                     // clear screen, cursor line 1, pos 0
{
    if (i2cLCD)                                                         // Johannes
        i2c_clear_OLED();
    else
    {
        delay(LCDdelay);
        uartWrite(0xFE);
        delay(LCDdelay);
        uartWrite(0x01);                                                // Clear
    }
    LCDline1();
}

void LCDline1(void)                                                     // Sets LCD Cursor to line 1 pos 0
{
    if (i2cLCD)                                                         // Johannes
        i2c_clr_line(7);
    else
    {
        delay(LCDdelay);
        uartWrite(0xFE);
        delay(LCDdelay);
        uartWrite(0x80);                                                // Line #1 pos 0
        delay(LCDdelay);
    }
}

void LCDline2(void)                                                     // Sets LCD Cursor to line 2 pos 0
{
    if (i2cLCD)                                                         // Johannes
        i2c_clr_line(8);
    else
    {
        delay(LCDdelay);
        uartWrite(0xFE);
        delay(LCDdelay);
        uartWrite(0xC0);                                                // Line #2
        delay(LCDdelay);
        printf("               ");                                      // Clear Line #2
        delay(LCDdelay);
        uartWrite(0xFE);
        delay(LCDdelay);
        uartWrite(0xC0);                                                // Line #2
        delay(LCDdelay);
    }
}

void cliProcess(void)
{
    char dummy;
    if (!cliMode)
    {
        cliMode = 1;
        uartPrint("\r\nEntering CLI Mode, type 'exit' or 'save' to return, or 'help' \r\n\r\n");
        cliVersion(&dummy);
        uartPrint("\r\n\r\n");
        cliHelp(&dummy);
        cliPrompt();
    }

    while (uartAvailable())
    {
        uint8_t c = uartRead();
        if (c == '\t' || c == '?')
        {
            // do tab completion
            const clicmd_t *cmd, *pstart = NULL, *pend = NULL;
            int i = bufferIndex;
            for (cmd = cmdTable; cmd < cmdTable + CMD_COUNT; cmd++)
            {
                if (bufferIndex && (strncasecmp(cliBuffer, cmd->name, bufferIndex) != 0))
                    continue;
                if (!pstart)
                    pstart = cmd;
                pend = cmd;
            }
            if (pstart)      /* Buffer matches one or more commands */
            {
                for (; ; bufferIndex++)
                {
                    if (pstart->name[bufferIndex] != pend->name[bufferIndex])
                        break;
                    if (!pstart->name[bufferIndex])
                    {
                        /* Unambiguous -- append a space */
                        cliBuffer[bufferIndex++] = ' ';
                        break;
                    }
                    cliBuffer[bufferIndex] = pstart->name[bufferIndex];
                }
            }
            if (!bufferIndex || pstart != pend)
            {
                /* Print list of ambiguous matches */
                uartPrint("\r\033[K");
                for (cmd = pstart; cmd <= pend; cmd++)
                {
                    uartPrint(cmd->name);
                    uartWrite('\t');
                }
                cliPrompt();
                i = 0;    /* Redraw prompt */
            }
            for (; i < bufferIndex; i++)
                uartWrite(cliBuffer[i]);
        }
        else if (!bufferIndex && c == 4)
        {
            cliExit(cliBuffer);
            return;
        }
        else if (c == 12)
        {
            // clear screen
            uartPrint("\033[2J\033[1;1H");
            cliPrompt();
        }
        else if (bufferIndex && (c == '\n' || c == '\r'))
        {
            // enter pressed
            clicmd_t *cmd = NULL;
            clicmd_t target;
            uartPrint("\r\n");
            cliBuffer[bufferIndex] = 0; // null terminate

            target.name = cliBuffer;
            target.param = NULL;

            cmd = bsearch(&target, cmdTable, CMD_COUNT, sizeof cmdTable[0], cliCompare);
            if (cmd)
                cmd->func(cliBuffer + strlen(cmd->name) + 1);
            else
                cliErrorMessage();

            memset(cliBuffer, 0, sizeof(cliBuffer));
            bufferIndex = 0;

            // 'exit' will reset this flag, so we don't need to print prompt again
            if (!cliMode)
                return;
            cliPrompt();
        }
        else if (c == 127)
        {
            // backspace
            if (bufferIndex)
            {
                cliBuffer[--bufferIndex] = 0;
                uartPrint("\010 \010");
            }
        }
        else if (bufferIndex < sizeof(cliBuffer) && c >= 32 && c <= 126)
        {
            if (!bufferIndex && c == 32)
                continue;
            cliBuffer[bufferIndex++] = c;
            uartWrite(c);
        }
    }
}

// ************************************************************************************************************
// DUMP THE LOG FOR DEBUG
// ************************************************************************************************************
static void cliGpslog(char *cmdline)
{
    int32_t  LATd, LONd, ALTd;
    int16_t  HDGd;
    uint16_t NRd = 0;

    printf("\r\nDisplaying %d Datasets\r\n\r\n", cfg.FDUsedDatasets);
    if(!GPSFloppyInitRead()) return;

    printf("\r\nThese are the basevalues\r\n");
  
    LATd = cfg.WP_BASE[LAT] << 6;
    LONd = cfg.WP_BASE[LON] << 6;
    ALTd = ((int32_t)cfg.WP_BASE_HIGHT << 6) / 100;
    HDGd = 0;
    printf("NR:%03u, LAT %d, LON %d, ALT %06d, HDG %d\r\n\r\n", NRd, LATd, LONd, ALTd, HDGd);

    while(ReadNextFloppyDataset(&NRd, &LATd, &LONd, &ALTd, &HDGd))
        printf("NR:%03u, LAT %d, LON %d, ALT %d, HDG %d\r\n", NRd + 1, LATd, LONd, ALTd / 100, HDGd);
}

// ************************************************************************************************************
// TestScan on the I2C bus
// ************************************************************************************************************
#define MMA8452_ADDRESS       0x1C
#define HMC5883L_ADDRESS      0x1E  // 0xA
#define DaddyW_SONAR          0x20  // Daddy Walross Sonar
#define EagleTreePowerPanel   0x3B  // Eagle Tree Power Panel
#define OLED1_address         0x3C  // OLED at address 0x3C in 7bit
#define OLED2_address         0x3D  // OLED at address 0x3D in 7bit
#define ADXL345_ADDRESS       0x53
#define BMA180_adress         0x64  // don't respond ??
#define MPU6050_ADDRESS       0x68  // 0x75     or 0x68  0x15
#define L3G4200D_ADDRESS      0x68  // 0x0f
#define BMP085_I2C_ADDR       0x77  // 0xD0
#define MS5611_ADDR           0x77  // 0xA0

/*
new May 15 2013 Johannes && Some stuff from me as well :)
*/
static void cliScanbus(char *cmdline)
{
    bool    ack;
    bool    msbaro   = false;
    bool    L3G4200D = false;
    uint8_t address;
    uint8_t nDevices;
    uint8_t sig = 0;
  	uint8_t bufdaddy[2];                                // Dummy for DaddyW testread
    char    buf[20];

    printf("\r\nScanning I2C-Bus\r\n\r\n");

    nDevices = 0;
    for(address = 1; address < 127; address++ )
    {
        ack = i2cRead(address, address, 1, &sig);       // Do a blind read. Perhaps it's sufficient? Otherwise the hard way...
      
        if (!ack && address == MMA8452_ADDRESS)         // Do it the hard way, if no ACK on mma Adress
        {
            sig = 0;
            i2cRead(MMA8452_ADDRESS, 0x0D, 1, &sig);
            if (sig == 0x2A || sig == 0x1A)
                ack = true;
            else
                ack = false;
        }

        if (!ack && address == DaddyW_SONAR)            // Do it the hard way, if no ACK on DaddyW Adress
        {
            ack = i2cRead(DaddyW_SONAR, 0x32, 2, bufdaddy);
        }
        
        if (!ack && address == MS5611_ADDR)             // MS Baro needs special treatment BMP would have said "ack" already
        {
            ack = i2cRead(MS5611_ADDR, 0xA0, 1, &sig);  // Sig is irrelevant?
            msbaro = ack;
        }

        if (!ack && address == MPU6050_ADDRESS)         // Special case mpu and L3G4200D have same Adr.
        {
            sig = 0;
            i2cRead(MPU6050_ADDRESS, 0x0F, 1, &sig);
            if (sig == 0xD3)
            {
                ack = true;              
                L3G4200D = true;
            }
        }

        if (ack)
        {
            printf("I2C device at 0x");
            if (address<16) printf("0");
            printf("%x",address);
            switch (address)
            {
            case MMA8452_ADDRESS:                       // Detection altered
                strcpy(buf,"MMA8452");
                break;
            case HMC5883L_ADDRESS:
                strcpy(buf,"HMC5883L");
                break;
            case DaddyW_SONAR:                          // Daddy Walross Sonar added
                strcpy(buf,"DaddyW Sonar");							
                break;						
            case EagleTreePowerPanel:
                strcpy(buf,"EagleTreePowerPanel");
                break;
            case OLED1_address:
                strcpy(buf,"OLED");                     // i2c_OLED_init();
                break;
            case OLED2_address:
                strcpy(buf,"OLED2");                    // i2c_OLED_init();
                break;
            case ADXL345_ADDRESS:                       // ADXL added
                strcpy(buf,"ADXL345");
                break;						
            case BMA180_adress:                         // Sensor currently not supported by a driver
                strcpy(buf,"BMA180");
                break;
            case MPU6050_ADDRESS:
                if (L3G4200D) strcpy(buf,"L3G4200D");
                else strcpy(buf,"MPU3050/MPU6050");
                break;
            case BMP085_I2C_ADDR:
                if(msbaro) strcpy(buf,"MS5611");
                else strcpy(buf,"BMP085");
                break;
            default:                                    // Unknown case added
                strcpy(buf,"UNKNOWN TO ME");
                break;
            }
            printf(" probably %s \r\n",buf);
            nDevices++;
        }
        delay(50);
    }
    uartPrint("\r\n");
    if (!nDevices) printf("No I2C devices\r\n");
    else printf("%d Devices\r\n",nDevices);
}

// ************************************************************************************************************
// More or less Dumb passthrough for gps config - Hacky but working
// Maybe we miss a byte or something but GPS communication is checksummed, so GPS and Tool will keep it valid.
// ************************************************************************************************************
static void cliPassgps(char *cmdline)
{
    uint8_t  serbyte, len, i;
    uint32_t wantedbaud = 0;
    bool     HaveMTK = false;

    if (!feature(FEATURE_GPS))                              // Don't ask for sensors(gps) here, it may not be initialized
    {
        printf("Enable Feature GPS first.\r\n");            // This will remind user to activate that feature, the friendly way.
        return;                                             // To prevent that: "I set it up in ucenter and now - nothing"
    }

    len = strlen(cmdline);    
    if (!len)
    {
        printf("Need option\r\n");
        printf("Writing ubx conf with different baudsetting must fail.\r\n");
        printf("Set Baud of planned config now. Repower after ucenter.\r\n");
        printf("MTK go with '0', set type to NMEA, set Baud of FW.\r\n\r\n");
        printf("Select Ublox Options\r\n\r\n");
        printf("0 No Options. All GPS\r\n");
        printf("1 UBX Force Sgnlstrngth\r\n");
        printf("2 UBX 115K Baud\r\n");
        printf("3 UBX  57K Baud\r\n");
        printf("4 UBX  38K Baud\r\n");
        printf("5 UBX  19K Baud\r\n\r\n");
        if (cfg.gps_type == 2 || cfg.gps_type == 3)         // GPS_NMEA = 0, GPS_UBLOX = 1, GPS_MTK16 = 2, GPS_MTK19 = 3, GPS_UBLOX_DUMB = 4
            printf("Actual MTK 57K Baud.\r\n");
        else
        {
            if (cfg.gps_type == 0)
                printf("Actual NMEA");
            else
                printf("Actual UBLOX");
            printf(" %d Baud.\r\n", cfg.gps_baudrate);
        }
        return;                                             // Bail out
    }
    else
    {
        if (cfg.gps_type == 2 || cfg.gps_type == 3) HaveMTK = true;

        if (cmdline[0] != '0' && HaveMTK)
        {
            cliErrorMessage();
            return;
        }
        switch(cmdline[0])
        {
        case '0':
            if (HaveMTK) wantedbaud = 57600;
             else wantedbaud = cfg.gps_baudrate;
            break;
        case '1':
            wantedbaud = cfg.gps_baudrate;
            UblxSignalStrength();
            break;
        case '2':
            wantedbaud = 115200;
            break;
        case '3':
            wantedbaud = 57600;
            break;
        case '4':
            wantedbaud = 38400;
            break;
        case '5':
            wantedbaud = 19200;
            break;
        default:
            cliErrorMessage();
            return;
        }
    }

    if(!HaveMTK)
    {
        if (wantedbaud == cfg.gps_baudrate)
        {
            printf("Keeping GPS Baud: %d.", wantedbaud);
        }
        else
        {
            printf("Setting %d Baud.", wantedbaud);
            UbloxForceBaud(wantedbaud);
        }
    }

    printf("\r\nProceeding. Close Terminal.");
    delay(2000);
    HaveNewGpsByte = false;  
    serialInit(wantedbaud);                                 // Set USB Baudrate
    uart2Init(wantedbaud, GPSbyteRec, false);               // Set GPS Baudrate and callbackhandler
    i = 0;
    while (i < 5)
    {
        if (uartAvailable())
        {
            serbyte = uartRead();                           // Read from USB
            if (serbyte == '#')                             // Break out with
                i++;
            else
                i = 0;
            uart2Write(serbyte);                            // Write to GPS
            while (!uart2TransmitEmpty());                  // wait for GPS Byte to be send
            LED1_TOGGLE;
        }
        if (HaveNewGpsByte)
        {
            serbyte        = NewGPSByte;                    // Read from GPS
            HaveNewGpsByte = false;
            uartWrite(serbyte);                             // Write to USB
            LED0_TOGGLE;
        }
    }
    uartPrint("Rebooting");
    systemReset(false);
}

static void GPSbyteRec(uint16_t c)
{
    NewGPSByte = c;
    HaveNewGpsByte = true;
}

static void cliFlash(char *cmdline)
{
    printf("Close terminal & flash\r\n");
    delay(1000);
    systemReset(true);                                  // reboot to bootloader
}

static void cliErrorMessage(void)
{
    uartPrint("That was Harakiri, try 'help'");
}

// MAVLINK STUFF AFFECTING CLI GOES HERE
bool baseflight_mavlink_send_paramlist(bool Reset)
{
    static uint16_t i = 0;
    if(Reset)
    {
        i = 0;
        BlockProtocolChange = false;
        return true;                                    // Return status not relevant but true because the "Reset" was a success
    }
    BlockProtocolChange = true;                         // Block Autodetect during transmission
    baseflight_mavlink_send_singleparam(i);
    i++;
    if (i == VALUE_COUNT)
    {
        i = 0;
        BlockProtocolChange = false;                    // Allow Autodetection again
        return true;                                    // I am done
    }
    else return false;
}

void baseflight_mavlink_send_singleparam(int16_t Nr)
{
    uint8_t StrLength, MavlinkParaType = MAV_VAR_FLOAT;
    float   value = 0;
    char    buf[16];                                      // Always send 16 chars
    mavlink_message_t msg;

    if(Nr == -1 || Nr >= VALUE_COUNT) return;
  
    memset (buf, 0, 16);                                  // Fill with 0 For Stringtermination
    StrLength = min(strlen(valueTable[Nr].name), 16);     // Copy max 16 Bytes
    memcpy (buf, valueTable[Nr].name, StrLength);

    switch(valueTable[Nr].type)
    {
    case VAR_UINT8:
        value = *(uint8_t *)valueTable[Nr].ptr;
        MavlinkParaType = MAV_VAR_UINT8;
        break;
    case VAR_INT8:
        value = *(int8_t *)valueTable[Nr].ptr;
        MavlinkParaType = MAV_VAR_INT8;
        break;
    case VAR_UINT16:
        value = *(uint16_t *)valueTable[Nr].ptr;
        MavlinkParaType = MAV_VAR_UINT16;
        break;
    case VAR_INT16:
        value = *(int16_t *)valueTable[Nr].ptr;
        MavlinkParaType = MAV_VAR_INT16;
        break;
    case VAR_UINT32:
        value = *(uint32_t *)valueTable[Nr].ptr;
        MavlinkParaType = MAV_VAR_UINT32;
        break;
    case VAR_FLOAT:
        value = *(float *)valueTable[Nr].ptr;
        break;
    }
    mavlink_msg_param_value_pack(1, 200, &msg, buf, value, MavlinkParaType, VALUE_COUNT, Nr);
		baseflight_mavlink_send_message(&msg);
}

bool baseflight_mavlink_set_param(mavlink_param_set_t *packet)
{
    mavlink_message_t msg;
    uint16_t i;
    uint8_t  StrLength, k;
    bool     returnval = false;
    float    value;

    if (strcmp((*packet).param_id, "") == 0) return false;   // Filter Shit Message here
    for (i = 0; i < VALUE_COUNT; i++)
    {
        StrLength = min(strlen(valueTable[i].name), 16);     // Compare max 16 Bytes
        for (k = 0; k < StrLength; k++) if(valueTable[i].name[k] - (*packet).param_id[k]) break; // Eat this strcmp !!
        if (k == StrLength)                                  // Strings match here
        {
            value = (*packet).param_value;
            if ((value > valueTable[i].max) || (value < valueTable[i].min)) return false;

            switch(valueTable[i].type)
            {
            case VAR_UINT8:
                *(uint8_t *)valueTable[i].ptr  = (uint8_t)value;
                break;
            case VAR_INT8:
                *(int8_t *)valueTable[i].ptr   = (int8_t)value;
                break;
            case VAR_UINT16:
                *(uint16_t *)valueTable[i].ptr = (uint16_t)value;
                break;
            case VAR_INT16:
                *(int16_t *)valueTable[i].ptr  = (int16_t)value;
                break;
            case VAR_UINT32:
                *(uint32_t *)valueTable[i].ptr = (uint32_t)value;
                break;
            case VAR_FLOAT:
                *(float *)valueTable[i].ptr = value;
                break;
            }
            mavlink_msg_param_value_pack(1, 200, &msg, (*packet).param_id, value, (*packet).param_type, VALUE_COUNT, -1); // Report parameter back if everything was fine.
		        baseflight_mavlink_send_message(&msg);
            returnval = true;
        }
    }
    return returnval;
}

/*

bool baseflight_mavlink_send_paramlist(bool Reset)
{
    static  uint16_t i = 0;
    uint8_t StrLength, MavlinkParaType = MAV_VAR_FLOAT;
    float   value = 0;
    char    buf[16];                                      // Always send 16 chars
    mavlink_message_t msg;

    if(Reset){i = 0; return true;}                       // Return status not relevant but true because the "Reset" was a success

    memset (buf, 0, 16);                                 // Fill with 0 For Stringtermination
    StrLength = min(strlen(valueTable[i].name), 16);     // Copy max 16 Bytes
    memcpy (buf, valueTable[i].name, StrLength);

    switch(valueTable[i].type)
    {
    case VAR_UINT8:
        value = *(uint8_t *)valueTable[i].ptr;
        MavlinkParaType = MAV_VAR_UINT8;
        break;
    case VAR_INT8:
        value = *(int8_t *)valueTable[i].ptr;
        MavlinkParaType = MAV_VAR_INT8;
        break;
    case VAR_UINT16:
        value = *(uint16_t *)valueTable[i].ptr;
        MavlinkParaType = MAV_VAR_UINT16;
        break;
    case VAR_INT16:
        value = *(int16_t *)valueTable[i].ptr;
        MavlinkParaType = MAV_VAR_INT16;
        break;
    case VAR_UINT32:
        value = *(uint32_t *)valueTable[i].ptr;
        MavlinkParaType = MAV_VAR_UINT32;
        break;
    case VAR_FLOAT:
        value = *(float *)valueTable[i].ptr;
        break;
    }
    mavlink_msg_param_value_pack(1, 200, &msg, buf, value, MavlinkParaType, VALUE_COUNT, i);
		baseflight_mavlink_send_message(&msg);
    i++;
    if (i == VALUE_COUNT){i = 0; return true;}          // I am done
    else return false;
}



// MAVLINK STUFF AFFECTING CLI GOES HERE
bool baseflight_mavlink_send_param(bool Reset)
{
    static  uint16_t i = 0;
    uint8_t StrLength, MavlinkParaType = MAV_VAR_FLOAT;
    char    buf[16];                                     // Always send 16 chars
    mavlink_message_t msg;
    union                                                // Put stuff together
    {
        int32_t  sig;
        uint32_t usig;
        float    flt;
		    uint8_t  bytes[4];
    } mavval;
    mavval.flt = 0;
    
    if(Reset){i = 0; return true;}                       // Return status not relevant but true because the "Reset" was a success

    memset (buf, 0, 16);                                 // Fill with 0 For Stringtermination
    StrLength = min(strlen(valueTable[i].name), 16);     // Copy max 16 Bytes
    memcpy (buf, valueTable[i].name, StrLength);

    switch(valueTable[i].type)
    {
    case VAR_UINT8:
        mavval.usig = *(uint8_t *)valueTable[i].ptr;
        MavlinkParaType = MAV_VAR_UINT8;
        break;
    case VAR_INT8:
        mavval.sig  = *(int8_t *)valueTable[i].ptr;
        MavlinkParaType = MAV_VAR_INT8;
        break;
    case VAR_UINT16:
        mavval.usig = *(uint16_t *)valueTable[i].ptr;
        MavlinkParaType = MAV_VAR_UINT16;
        break;
    case VAR_INT16:
        mavval.sig  = *(int16_t *)valueTable[i].ptr;
        MavlinkParaType = MAV_VAR_INT16;
        break;
    case VAR_UINT32:
        mavval.usig = *(uint32_t *)valueTable[i].ptr;
        MavlinkParaType = MAV_VAR_UINT32;
        break;
    case VAR_FLOAT:
        mavval.flt  = *(float *)valueTable[i].ptr;
        break;
    }
    mavlink_msg_param_value_pack(1, 200, &msg, buf, mavval.flt, MavlinkParaType, VALUE_COUNT, i); // Always transmit as "flt"
		baseflight_mavlink_send_message(&msg);
    i++;
    if (i == VALUE_COUNT){i = 0; return true;}          // I am done
    else return false;
}

bool baseflight_mavlink_set_param(mavlink_param_set_t *packet)
{
    mavlink_message_t msg;
    uint16_t i;
    uint8_t  StrLength, k;
    bool     returnvalue = false;
    union                                                // Put stuff together
    {
        int32_t  sig;
        uint32_t usig;
        float    flt;
		    uint8_t  bytes[4];
    } mavval;
    mavval.flt = 0;

    if (strcmp((*packet).param_id, "") == 0) return false;   // Filter Shit Message here
    for (i = 0; i < VALUE_COUNT; i++)
    {
        StrLength = min(strlen(valueTable[i].name), 16);     // Compare max 16 Bytes
        for (k = 0; k < StrLength; k++) if(valueTable[i].name[k] - (*packet).param_id[k]) break; // Eat this strcmp !!
        if (k == StrLength)                                  // Strings match here
        {
            mavval.flt = (*packet).param_value;
            if (( mavval.flt > valueTable[i].max) || ( mavval.flt < valueTable[i].min)) return false;

            switch(valueTable[i].type)
            {
            case VAR_UINT8:
                *(uint8_t *)valueTable[i].ptr  = (uint8_t) mavval.usig;
                break;
            case VAR_INT8:
                *(int8_t *)valueTable[i].ptr   = (int8_t) mavval.sig;
                break;
            case VAR_UINT16:
                *(uint16_t *)valueTable[i].ptr = (uint16_t)mavval.usig;
                break;
            case VAR_INT16:
                *(int16_t *)valueTable[i].ptr  = (int16_t)mavval.sig;
                break;
            case VAR_UINT32:
                *(uint32_t *)valueTable[i].ptr = (uint32_t)mavval.usig;
                break;
            case VAR_FLOAT:
                *(float *)valueTable[i].ptr = mavval.flt;
                break;
            }
            mavlink_msg_param_value_pack(1, 200, &msg, (*packet).param_id, mavval.flt, (*packet).param_type, VALUE_COUNT, -1); // Report parameter back if everything was fine.
		        baseflight_mavlink_send_message(&msg);
            returnvalue = true;
        }
    }
    return returnvalue;
}

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
