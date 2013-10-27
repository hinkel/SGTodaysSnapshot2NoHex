#include "board.h"
#include "mw.h"

/*
typedef enum FloppyDiskType
{
    FD_MODE_NONE = 0,
    FD_MODE_GPSLOGGER,
    FD_MODE_WPLIST,
    FD_MODE_IMULOGGER
} FloppyDiskType;
....
    int32_t  WP_BASE[2];                    // Base LAT/LON Coordinates for WP stuff
    int16_t  WP_BASE_HIGHT;                 // Starthight, normally "0"
    uint16_t FDUsedDatasets;                // Number of valid datasets of current type
    int8_t   FloppyDisk[2340];              // Reserve 2200 general purpose bytes
....
FD_MODE_GPSLOGGER

Basic Function
==============
The LOGGER just works with Byte deltas and reduce the resolution of the accuracy.
The initial Dataset is complete but reduced in accuracy
Lat     /  32 ErrorLAT = 32 * MagicEarthNumber = +-35,6 cm NOTE: Division is done by bitshift "GpsShift"
Lon     /  32 ErrorLON = 1/cos(lat) * ErrorLAT // so Lon = Lat error at equator, increasing to the poles
Alt     /  32 ErrorALT = +-32 cm NOTE: Division is done by bitshift "AltShift"
Heading /   2 Stores +-180 Deg with +-2 degree resolution. So no "delta" here

SpeedLimits at o,5 Hz (1 Dataset every 2 seconds)
================================================
Horizontal Speed LIMIT
======================
Maximal GPS delta is Byte (8Bit) + 5Bit (that div 32 stuff) - 1Bit for sign = 12 Bit
That is 4096 * MagicEarthNumber("1,113195") = 45,6 cm/2 sec = 22,8 m/s = 82 Km/h.
So logging beyond 82 Km/h will tilt out.

Vertical Speed LIMITS
=====================
Maximal delta in cm between readings is 8Bit + 5Bit - 1Bit for sign = 12 Bit
That is 4096/2 sec = 20,5 m/s = 74 Km/h
Maximal hight is limited to 16Bit + 5Bit - 1SignBit = 20Bit = +-10,5 KM

YAW Speed
=========
Not limited, since heading is stored without delta. Resolution is reduced to 2 degree.

Duration of Logging at 0,5 Hz and current bytesize
==================================================
Dataset = 4 Bytes and (FDByteSize)2340 Bytes available = 585 Datasets at 1/0.5s = 1170 sec = 19 Min 30 sec
*/

#define GPSLoggerDatasetSize 4
#define MaxDataSetNr FDByteSize / GPSLoggerDatasetSize                               // Set the maximal number of datasets

static uint16_t CurrentDatasetToRead;
static int32_t  LastGPS64[2] = { 0, 0};
static int16_t  LastALT64 = 0;
int8_t bytes[GPSLoggerDatasetSize];                                                  // That "union" stuff wasn't better so ditched for my dumb version
#define LATDELTA64Off 0
#define LONDELTA64Off 1
#define ALTDELTA64Off 2
#define HEADING2Off   3
#define GpsShift 5                                                                   // That equals to a div or mult. with 2^Shiftvalue
#define AltShift 5                                                                   // That equals to a div or mult. with 2^Shiftvalue

//
// This initializes sequential write to array
// Turns false on Error
//
bool GPSFloppyInitWrite(void)
{
    bool output = false;

    switch(cfg.floppy_mode)                                                          // Usagemode of free Space. 1 = GPS Logger ... more to come?
    {
    case FD_MODE_GPSLOGGER:
        cfg.FDUsedDatasets = 0;                                                      // Declare empty buffer
        if (f.GPS_FIX && GroundAltInitialized && f.ARMED && MaxDataSetNr > 0)        // Only turn true if everything is fine
        {
            LastGPS64[LAT]        = Real_GPS_coord[LAT] >> GpsShift;
            cfg.WP_BASE[LAT]      = LastGPS64[LAT];
            LastGPS64[LON]        = Real_GPS_coord[LON] >> GpsShift;
            cfg.WP_BASE[LON]      = LastGPS64[LON];
            LastALT64             = (int16_t)((int32_t)EstAlt >> AltShift);
            cfg.WP_BASE_HIGHT     = LastALT64;
            ScheduleEEPROMwriteMS = 1;                                               // eeprom write on next disarm to store the upcoming data
            output = true;
        }
    default:                                                                         // More to come
        break;
    }
    return output;
}

//
// Does the sequential write of Dataset on each call until floppy is full
// Turns false on Error
// 
bool WriteNextFloppyDataset(void)
{
    bool     output = false;
    int32_t  actual32;
    int16_t  actual16;
    uint8_t  i;
    uint16_t ByteOffset;

    switch(cfg.floppy_mode)
    {
    case FD_MODE_GPSLOGGER:
        if (f.GPS_FIX && f.ARMED && cfg.FDUsedDatasets < MaxDataSetNr && MaxDataSetNr != 0)
        {
            actual32             = Real_GPS_coord[LAT] >> GpsShift;
            bytes[LATDELTA64Off] = (int8_t)constrain(actual32 - LastGPS64[LAT], -128, 127);      // Delta pos moving north
            LastGPS64[LAT]       = actual32;
            actual32             = Real_GPS_coord[LON] >> GpsShift;
            bytes[LONDELTA64Off] = (int8_t)constrain(actual32 - LastGPS64[LON], -128, 127);      // Delta pos moving east
            LastGPS64[LON]       = actual32;
            actual16             = (int16_t)((int32_t)EstAlt >> AltShift);
            bytes[ALTDELTA64Off] = (int8_t)constrain(actual16 - LastALT64, -128, 127);           // Delta pos moving up
            LastALT64            = actual16;
            bytes[HEADING2Off]   = (int8_t)((int16_t)heading >> 1);
            ByteOffset           = cfg.FDUsedDatasets * GPSLoggerDatasetSize;                    // Get Dataset Offset
            for(i = 0; i < GPSLoggerDatasetSize; i++) cfg.FloppyDisk[ByteOffset + i] = bytes[i]; // Write Dataset
            cfg.FDUsedDatasets ++;
            output = true;
        }
    default:
        break;
    }
    return output;
}

//
// This initializes sequential read from array
// Turns false when no datasets
//
bool GPSFloppyInitRead(void)                                                         // This initializes sequential read from array
{
    bool output = false;

    switch(cfg.floppy_mode)                                                          // Usagemode of free Space. 1 = GPS Logger ... more to come?
    {
    case FD_MODE_GPSLOGGER:
        if (cfg.FDUsedDatasets != 0)
        {
            LastGPS64[LAT] = cfg.WP_BASE[LAT];
            LastGPS64[LON] = cfg.WP_BASE[LON];
            LastALT64      = cfg.WP_BASE_HIGHT;
            CurrentDatasetToRead = 0;
            output = true;
        }
    default:                                                                         // More to come
        break;
    }
    return output;
}

//
// Does the sequential read of Dataset on each call until everything is done
// The DataSetNr reported is counting up from ZERO! So max is cfg.FDUsedDatasets - 1
// Turns false on Error (No Dataset more to read)
// 
bool ReadNextFloppyDataset(uint16_t *DataSetNr, int32_t *DataLAT, int32_t *DataLON, int32_t *DataALT, int16_t *DataHDG)
{
    bool     output = false;
    uint16_t ByteOffset;
    uint8_t  i;

    switch(cfg.floppy_mode)                                                          // Usagemode of free Space. 1 = GPS Logger ... more to come?
    {
    case FD_MODE_GPSLOGGER:
        if (CurrentDatasetToRead < cfg.FDUsedDatasets  && cfg.FDUsedDatasets != 0)
        {
            ByteOffset = CurrentDatasetToRead * GPSLoggerDatasetSize;                // Get Dataset Offset
            for(i = 0; i < GPSLoggerDatasetSize; i++) bytes[i] = cfg.FloppyDisk[ByteOffset + i]; // Read Dataset to buffer
            *DataSetNr = CurrentDatasetToRead;
            LastGPS64[LAT] += (int32_t)bytes[LAT];
            LastGPS64[LON] += (int32_t)bytes[LON];
            LastALT64      += (int16_t)bytes[ALTDELTA64Off];
            *DataLAT        = LastGPS64[LAT]     << GpsShift;
            *DataLON        = LastGPS64[LON]     << GpsShift;
            *DataALT        = (int32_t)LastALT64 << AltShift;
            *DataHDG        = (int16_t)bytes[HEADING2Off] << 1;
            CurrentDatasetToRead++;                                                  // Next Dataset on the next run
            output = true;
        }
    default:                                                                         // More to come
        break;
    }
    return output;
}

// HERE FOLLOWS THE WP STORAGE STUFF ... LATER ...
