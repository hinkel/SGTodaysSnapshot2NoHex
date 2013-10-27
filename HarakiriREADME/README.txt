Naze32 Harakiri10 Summer Games pre2.6
=====================================
- Note: Used Hardware: MPU6050 / MS5611 / BMP085 / HMC5883 / GPS-NL-652ETTL / HC-SR04 / MB1200 XL (NAZE v4)
- Probably now compatible with v5.
- Probably some MULTITYPE_FLYING_WING stuff fixed in mixer.c see: http://code.google.com/p/afrodevices/source/detail?r=450
- Fixed: http://www.multiwii.com/forum/viewtopic.php?f=8&t=4150#p42211 (Throttle expo array could be out of bounds)
- Put Gyrosmoothing to floats
- Deleted FEATURE_GYRO_SMOOTHING added gy_smrll, gy_smptc, gy_smyw instead. Now you can adjust gy_smyw for tricopter.
- Added: "tri_ydel" Tricopter Yaw Delay in ms [0(disables)-1000]: Purpose: Avoids Yawservomovement when disarmed (sends "tri_yaw_middle" then), when doing yawarm there is a delay in initial yaw servo movement.
- Imu rearrangements
- L3G4200D driver fix like here http://code.google.com/p/afrodevices/source/detail?r=403
- HMC5883 Driverchanges. With mag_gain(0 default, use 1 on problematic copters) you can adjust the GAUS sensitivity for bootup gaincalculation. Recalibrate compass when changing that.
- Rework Magcalibration. Removed old calibration and variables: mag_oldctime and mag_oldcalib.
  mag_time (1-6) will set the mag calibration time in minutes now. Default is 1.
  Tested "intelligent mag calibrating" idea but results vary too much between setups/copters that it didn't work out. Idea still in mind, maybe later..
- Reworked Failsafe / cleanup / improved hoverthrottle for fs with barofunction. Since the throttlechannel is not valid in FS situations, a statistic average is taken gathered during the flight.
  If that average is not bigger than rc_min + 5% an error is assumed and the predefined fs_rcthr is taken as althold baselinethrottle.
  Thx to Hinkel for pointing out that subject here: http://www.multiwii.com/forum/viewtopic.php?f=23&t=3524&start=280#p41661
- Reworked ACC & GYRO Calibration to Sphere Algo. "moron_threshold" is set to more sensitive. Acc Calibration takes longer now (more values).
- Reworked Temperature readout for MPU6050 & MPU3050 (preparation for temperature compensation). Added some code for correct temperature / alternative MS5611 calculation (both unused and optional - needs recompile).
- Inserted DEBUGMODE as #define in the code for some sensors. Reachable via CLI (needs recompile). Spits out some internal calibration data.
- Reorganized mainprogram, put throttlechannel/pid attenuation calculation into the rc loop
- Removed tiny bug in cli auxset: removing a not set box would set it. Improved code.

Althold:
- Sonar/Baro code rework/cleanup/Sonardriver. Sonaroffset generation improved with averaging.
- SONAR hardwareproblem info/found:
* Sudden errors with PWM Sonar readout. Keep Sonar pwm signal clear, use a ferrite and/or shielded dataline.
  Check your proper sonar function before first flight if you altered configuration with snr_debg = 1.
* Worst case: A freezing sonar with a valid value in flight will result in an althold doing a rapid climb.
  Looking into a softwareway to check for "in range" but actual invalid data. SOLVED: Disconnect for PWM Sonar is detected in flight!
  So if the sonar readout is dead for a second the Sonar sensor is disabled for that session, no matter if a random reconnect happens. You might see a rapid climb during
  that detection second, then everything is back to "normal" (without sonar!). I can not try that out because I don't know how to produce a faulty
  on/off connection on purpose. But if you see that behaviour in flight, you can be shure that you have managed to produce a slack joint in your sonar wiring.
  Note: With snr_debg = 1 you will still see the last sonar values before the sensor is shut down, these values will *not* be taken into account for the rest of the flight.
- Added, tested, deleted the arducopter groundpressure/groundtemperature stuff for baro like suggested by pm1. Could not compensate for misreadings due to baroheatup, worsens althold from what I've seen in my tests.
  Maybe further investigation with different code (Ms5611 "T2" temp correction) but currently no priority.
- Baro/Sonar Landingrate is now in cm/s (see below "Changed Var Values")
- Engaging Autoland is smoother due to suppression of clearing I.
- Baro Automodes: Autloand and Autostart turn off Horizon and turn on Anglemode automatically, if not already selected.
- Autostart. It is implemented here and turned off by default for future auto missions.
  Usage: When armed and copter not flying and Baromode is on and the throttle is put to the middle (incl. "rc_dbah") the autostart sequence is launched. It can be aborted by moving the throttle out of the middle zone.
  On abortion it will do an althold and wait for throttle recenter for further altholdchanges. Autostart can not be triggered in flight.
  Variables:
  "set as_trgt  = X"  X =  0 - 15 (m) DEFAULT:0. 0 Disables Autostart. Targethight is in m. Note: use 2m or more for reliable (hight) operation. 1m will work but overshoot.
  "set as_lnchr = X   X = 50 - 250 no dimension DEFAULT:200  Autostart launchrate to get off the ground. When as_stdev is exceeded, as_clmbr takes over
  "set as_clmbr = X"  X = 50 - 250cm/s DEFAULT:100. Autostart climbrate in cm/s after liftoff! Autostart Rate in cm/s will be lowered when approaching targethight.
  "set as_stdev = X"  X =  5 - 20 no dimension DEFAULT:10. This is the std. deviation of the variometer when a liftoff is assumed. The higher the more unsensitive.
  It is highly recommended to "set esc_nfly = x" (X = 0 as default) to an appropiate PWM value. It's the PWM value where the copter barely does NOT fly. In my case it's 1300.
  Purpose: The Copter will decide upon it's airborne status, if that PWM value is exceeded once. If it is left at 0 a value of 5% above esc_min is assumed.
  It will affect the initial throttle for Autostart. It will affect the disarm timeout when autolanding. It is used for initial hover throttle plausibility check in failsafe.
  Just for info: Internal function of Autostart: It is done in 3 stages.
  1. Initialize real Targethight, apply esc_nfly as basic throttle.
  2. Increase a virtual Targethight (beyond the real targethight) at desired rate until 50cm/s is achieved or the vario std. deviation is exceeded. So a liftoff is assumed and a short althold is engaged to reset virtual targethight.
     (The virtual targethight works like a rubberband to pull the copter off the ground)
  3. Now the real thing: Proceed with desired rate to actual Targethight.
  WARNING: Autostart is highly *NOT* recommended for indoor use! It will break the ceiling! Stand back on Autostart it *WILL* fly into your face.

GPS:
- Added Timejitter Filter for 20Hz and 50Hz GPS Datarates (preparation for new skytraq GPS)
- Added rtl_cr [10 - 200cm/s DEFAULT:80] When rtl_mnh (min hight for RTL) is defined this is the climbrate in cm/s that is used.

MSP:
- Added "OSD SW" for KV_Team_OSD, thanks Hinkel for the input: http://www.multiwii.com/forum/viewtopic.php?f=23&t=3524&start=350#p42658
- Added MSP_STATUS profile 0 like suggested by disq http://www.multiwii.com/forum/viewtopic.php?f=23&t=3524&start=320#p42354

RC:
- Added serial Graupner SumH support from here: http://fpv-treff.de/viewtopic.php?f=18&t=1368&start=3020#p44535 (some changes for integration made)
- Added Devo RSSI output (mwii & mavlink protocol) for details ask Bamfax :)
  here: http://fpv-treff.de/viewtopic.php?f=18&t=2181, http://fpv-community.de/showthread.php?33343-DSM-ohne-Spektrum-mit-SumPPM-12-Kan%E4len-und-RSSI
- Added a cutoff for rssi "set rssicut X" [0-80%][0 Disables/Default] Below that percentage rssi will show zero.
- Did not implement rssi for failsave because if the rssi is reported wrong (cable, devocode etc.) and the user is still in control a fs wouldn't be too great.

Setup & Motor/ESC help:
- Slight change of feature pass. The passmotor is always reset to 0 (use all motors) on de/activation of feature pass.
- Added Motorstatistics (see MotorStats.gif) to show average values for max 8 Motors. They are NOT stored. After a flight you can see in cli (type "status"):
-- Motornumber (like shown in the BF Manual)
-- The relative load of each motor during flight in percent, absolute throttle not taken into account here (total flightthrottle is taken as 100%).
-- The absolute PWM val for each motor including minthrottle.
-- The relative motorload in percent in relation to the throttlerange, to see how much headroom is left.
  Purpose: On test flights of a new setup you can see if motors are stressed more in an load unbalanced copter.
  Note: Motordata are gathered at 10Hz rate. The percentages are calculated without floatpoint or correct rounding because it's not about 0.x% precision here.
  Note: ESC RPM response maybe not linear to PWM input (that is measured here for display). Depends on ESC.
  Note: I don't know if this is useful at all, since i have only symetric X - shape quadrocopters that are easy to balance. If you own a copter with some spidertype
        frame, please report back if this is useful or not.

Deleted:
- Deleted U_ID_0.
- Deleted BOXCAMTRIG (no hardwarepin or code behind that anyway)
- Deleted FY90Q drv_pwm_fy90q.c, drv_adc_fy90q.c, drv_i2c_soft.c
- Deleted "FEATURE_POWERMETER" because there was/is no code behind it.
- Deleted FEATURE_GYRO_SMOOTHING (replaced)
- Deleted mag_oldctime and mag_oldcalib (replaced)
- Deleted cli function "aux" because it is replaced by human readable "auxset"

Changed Variablenames:
(Purpose:
1: Sort/group by functionality. Where possible, the first 2-3 chars should represent the MAIN purpose / affiliation followed by a "_".
That's the basic idea behind that.
2: Try to stuff into 8 chars for better mavlink- and generaldisplay.
These goals will not be possible with all variables.
Still boring work ahead, but had to be started sometime.)

OLD                          NEW
maxcheck                  -> rc_max
mincheck                  -> rc_min
midrc                     -> rc_mid
killswitchtime            -> rc_killt
auxChannels               -> rc_auxch
retarded_arm              -> rc_rllrm (Arming/Disarming with roll is dangerous when doing flips!)
deadband                  -> rc_db
yawdeadband               -> rc_dbyw
alt_hold_throttle_neutral -> rc_dbah
gps_adddb                 -> rc_dbgps
failsafe_delay            -> fs_delay
failsafe_off_delay        -> fs_ofdel
failsafe_throttle         -> fs_rcthr
failsafe_deadpilot        -> fs_ddplt
failsafe_justph           -> fs_jstph
failsafe_ignoreSNR        -> fs_nosnr
snr_debug                 -> snr_dbg
minthrottle               -> esc_min
maxthrottle               -> esc_max
mincommand                -> esc_moff
al_lndthr                 -> esc_nfly (ESC NOFLY, is a copter dependant value greater than esc_min. It is important in autostart, landing disarm and failsafe plausibility throttlecheck. If 0 esc_min+5% is taken)
motor_pwm_rate            -> esc_pwm
servo_pwm_rate            -> srv_pwm
passmotor                 -> pass_mot
mag_declination           -> mag_dec
gyro_cmpf_factor          -> gy_cmpf
gyro_cmpfm_factor         -> gy_cmpfm
gyro_lpf                  -> gy_lpf
moron_threshold           -> gy_stdev (Allowed Standard Deviation during gyro initialization)
acc_hardware              -> acc_hdw
acc_lpf_factor            -> acc_lpf
acc_ins_lpf               -> acc_ilpf
accz_vel_cf               -> accz_vcf
accz_alt_cf               -> accz_acf
yaw_direction             -> tri_ydir
tri_yaw_middle            -> tri_ymid
tri_yaw_min               -> tri_ymin
tri_yaw_max               -> tri_ymax
triywdel                  -> tri_ydel
barodownscale             -> bar_dscl
baro_lag                  -> bar_lag
baro_debug                -> bar_dbg
gimbal_pitch_gain         -> gbl_pgn
gimbal_roll_gain          -> gbl_rgn
gimbal_flags              -> gbl_flg
gimbal_pitch_min          -> gbl_pmn
gimbal_pitch_max          -> gbl_pmx
gimbal_pitch_mid          -> gbl_pmd
gimbal_roll_min           -> gbl_rmn
gimbal_roll_max           -> gbl_rmx
gimbal_roll_mid           -> gbl_rmd
gps_rtl_minhight          -> rtl_mnh
gps_rtl_mindist           -> rtl_mnd


Changed Var Values:
mainpidctrl = X   0 = Altered Original, 1 = New controller (was 1 & 2 before)
al_barolr & al_snrlr // [10 - 200cm/s DEFAULT:50] Baro/Sonar Landingrate


Naze32 Harakiri10 Summer Games2.5
=================================
- Experimental IMU Changes. Better MPU Gyro usage. All acc scaled to "512" for 1G
- Precision improved ACC/Gyro calibration.
- MPU6050 "single shot" readout like suggested by Sebbi
- Changed MPU Acc setting from 8G to 4G, increasing resolution without observing saturation effect (right now...).
- Experimental IMU Change for GPS
- gyro_cmpf_factor changed to 1000 (from 400)
- Added vario data transmission within MSP_ALTITUDE


Naze32 Harakiri10 Summer Games2.4 WITH EXPERIMENTAL LOGGING
===========================================================
- LLIGHTS & LEDMAX Deleted (Had no use in BF anyway)

// Logging don't expect anything of it yet
// - Introduced some "GPS LOG Box" will record when armed, only one log possible. So restarting logging will delete old log.
// - Introduced gpslog in cli to show dataset(s)
// - GPS Logging for 19,5 Minutes for LAT/LON/ALT/HEADING at 0.5 HZ (every 2 sec new dataset)
// - Logging precision is slightly decreased for compression reasons, one Dataset needs just 4 Bytes, so 2,3KB can store 20Min flight
// - Flightstats couldn't display negative alt, basic stats can be saved now. When doing a general save on eeprom (like save in cli or write in gui, or if logging anyway)
// - Flightstats will be cleared at power up with the default stat_clear = 1. With 0 the last saved stats will be taken into account concerning max speed/hight etc..

Core changes:
- Reintroduced old, (t)rusty MTK parser.
- Removed the fix acc_1G value from multiwii and calculate real acc_1G during calibration. So you will not see that fix "512" for mpu any more. On the first run altitude will show a flatline
  until ACC calibrated (wait a little for save) and FC repowered
- Moved Gyro calibration, angle calculation to float point calculations.
- INS FACTORS WILL HAVE TO BE RETUNED - OMG. gps_ins_vel reduced to 0.6. nav_slew_rate set to 20 now (reducing, increases strength). accz_vel_cf untouched seems to fit.
- gps_tbe_speed deleted was of limited use
- PH/RTL Bug fixed (actual position could be ignored esp. on RTH)
- PosrI put to work (and scaled further down by /100) but just with the velocity error that is calculated from position error (posP). So will hopefully have effect now without circeling. Default 0.Untested.
- Stay away from the feature inflightcalibration it will probably kill INS/ACC functions because it isn't affecting the trims but the real acc calibration. - Outdated mwii code.
- Reworked MsBaroDriver a little (probably slight resolution increase)

Naze32 Harakiri10 Summer Games2.3
=================================
- Little update just concerning ublox parser. Some people reportet uBLox 6M didn't show Lat/Lon but satcount.
- So ublox parser is redone here and it is a mixture of current BF/Mwii and Arducopter driver
- I hope it resolves that issue. Works the same on my rig than the old one
- Reduced the defaultPIDs, because the correct controller is more aggressive

Naze32 Harakiri10 Summer Games2.2
=================================
- Just updated main PID controller (mainpidctrl = 1 (default)) according to BRM's suggestions here: http://www.multiwii.com/forum/viewtopic.php?f=23&t=3524&start=150#p38927
- Relaxed timings on ublox startup configuration
- Minor mavlink changes

Naze32 Harakiri10 Summer Games2.1
=================================
- Just updated the Arducopter "plain earth" bearing calculation to a little more STM like correct, spherical Bearing.
Forumla:http://www.movable-type.co.uk/scripts/latlong.html under "BEARING"
JavaScript: 	
var y = Math.sin(dLon) * Math.cos(lat2);
var x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1)*Math.cos(lat2)*Math.cos(dLon);
var brng = Math.atan2(y, x).toDeg();
I don't understand that but RTL works to the point now :)

Harakiri Summergames2
=====================

- Feature telemetry is gone and replaced by "tele_prot = X"
- Some basic Mavlinksupport. It's beta but there are a few things you can do already.
- Autosensing Mavlink/Multiwii protocols (can change every 4 seconds)
- Due to Autosensing you will see the Mavlink "1Hz Heartbeat - garbage" in cli.
- Entering the CLI requires three times "#" now! ("###"). Alternatively 3 times <RETURN>. CLI entering is only possible when disarmed.
- Flashing requires three times "R" now! ("RRR")
- Reworked PH PosrD Controller. PosrD is scaled down now (factor 30)
- Re-enabled crosstrack support. Crosstrack gain can be adjusted or turned off.


Telemetry / Mavlink
===================
Since "feature telemetry" was just for FRsky it was misleading. Some people might do multiwii/bt telemetry and are mislead by that.
Now you can "set tele_prot = X".
That X will decide, what is done on the usb/telem. port ONLY WHEN ARMED. When disarmed the mavlink/multiwii autodetection will kick in.
So here is what that X stands for:
0 (Dfault) = Keep Multiwii @CurrentUSB Baud (like always)
1          = Frsky @9600Baud Turn to Frsky protocol at 9600 Baud when armed.
2          = Mavlink @CurrentUSB Baud. Some Mavlink 1.0 Packages are sent at low rate (ca <2Kb/s)
3          = Mavlink @57KBaud (like stock minimOSD wants it) Sends the same stuff like in mode "2" but only at 57K

NOTE: If you choose to work just with missionplaner and also want to feed minimosd you might want to set serial_baudrate = 57600 and set
set tele_prot to 2 or 3. So you will not loose MP connection on arming. Or recompile MinimOSD FW with 115KBaud setting.

Current Datastream over mavlink:
Heartbeat      @ 1  Hz: Armed/Disarmed, Copter Type, Stabilized/Acro.
Sys_status     @ 2  Hz: Sensors present and healthstatus. Voltage.
Attitude       @30  Hz: TimeStamp(ms), roll/pitch/yaw(compass heading - RAD)
RC_Channels    @ 2  Hz: RAW, unscaled 8 RC Channels and RSSI (currently unknown to naze)
VFR_HUD        @10  Hz: Speed measured by GPS, scaled Throttle (0-100%), Baro Altitude, Variometer, compass heading (again, this time in Degree)
Scaled_pressure@ 0.5Hz: Gyro Temperature, Airpressure in hPa (mBar)

Note: The Datarates are wishful thinking, because only one Datapacket is send per (100Hz-)cycle to keep the serial rate low. So "Attitude"
is actually send at 25.xHz rate. The real rates are "as high" like the APM sends them. I checked it with GCS. Attitude is 10Hz at Arducopter, so that's faster now.
These Data are send without request, once Mavlink is established. MinimOSD is not tested but should be able to read and display something from the datastream.
Datastreamrequests are currently not handled (with one exception) so for testing you can connect minimOSD only with its' RX pin (same like in arducopter telem mode).
Let me know if minimOSD works with that (set tele_prot = 3).
The only request that is handled is, when Missionplaner requests Parameterlist (won't work with the windows GCS soft I tested).
Missionplaner will load the Parameterlist on start. It will be accessible as advantaged parameter list. You can change and write and compare whole parametersets.
Note: New MP will cry out for missing Arducopterparameters. The new MP doesn't show altitude and some other data on mainscreen - they changed that.
My old and rusty Mission Planner 1.2.38 shows everything and is not so picky. So go for older MP.
Note: Naze parameters will be crippled to 16 chars if necessary. No problem.
WP etc is not supported right now. The main reason for that is that i am having trouble with Naze EEPROM writing more than 1KB data.


Changes in PH
=============
Parameters used:
PosrP (Strength of velocity influence, tries to keep velocity, normally "0" in PH, but might change with PosP - see below)
PosrI (keep it 0, might lead to circeling)
PosrD is rescaled now.

PosP Works in conjunction with PosrP. And defines how much a POSITION error is translated to an VELOCITY error.

Parameters NOT USED: PosI and PosD

gps_ph_brkacc = 40 (Dfault)  // [1 - 500] Is the assumed negative braking acceleration in cm/(s*s) of copter. Value is positive though. It will be a timeout. The lower the Value the longe the Timeout.

gps_ph_abstub = 150 (Dfault) // 0 - 1000cm (150 Dfault, 0 disables) Defines the "bath tub" around current absolute PH Position, where PosP is diminished, reaction gets harder on tubs edge and then goes on linear.
I changed the form of the bathtub -> see attached picture. The bathtub is for absolute position (influence set by PosP).

gps_tbe_speed = 0(Dfault)    // 0 - 1000 (0 disables) Speed in cm/s for TBE detection MUST be greater than gps_ph_settlespeed or it will be disabled
When at the end of the PH chain a movement with that speed (150cm/s) is detected for 2 seconds a toilet bowl is assumed and the PH cascade is redone, keeping the target PH Position in mind.
Thanks "HINKEL" for the idea!
WARNING: gps_tbe_speed Also can disable PH in wind, because if it is carried away with 150cm/s a TBE will be assumed! Set it to 0 to disable it.


Changes in NAVIGATION
=====================
The final stage of a RTH is the PH. If you move the sticks and oversteer, the PH cascade is reset and a new GPS targetpoint is set. It will not land at the once assumed GPS pos anymore thats on purpose.

nav_tiltcomp is reduced to 20  // 0 - 100 (20 TestDefault) Only arducopter really knows. Dfault was 54. This is some kind of a hack of them to reach actual nav_speed_max. 54 was Dfault, 0 disables

nav_ctrkgain = 0.5  // 0 - 10.0 (0.5 TestDefault) That is the "Crosstrackgain" APM Dfault is "1". "0" disables
Re - Introduced Crosstrack. I think it is not neccessary for copters. You can disable it with "0".
See for details: http://diydrones.com/profiles/blogs/705844:BlogPost:43438





