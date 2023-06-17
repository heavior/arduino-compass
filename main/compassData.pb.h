/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.8-dev */

#ifndef PB_COMPASS_COMPASSDATA_PB_H_INCLUDED
#define PB_COMPASS_COMPASSDATA_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
typedef struct _compass_Coordinate {
    double latitude;
    double longitude;
} compass_Coordinate;

typedef struct _compass_MapPoint {
    uint32_t id;
    char name[256];
    uint32_t radius;
    bool has_coordinates;
    compass_Coordinate coordinates;
    bool visited;
} compass_MapPoint;

typedef struct _compass_CompassState {
    bool has_location;
    compass_Coordinate location; /* Current coordinates from GPS */
    bool havePosition; /* do we have gps reading or not */
    bool closed; /* closed lid (based on hall sensor) */
    int32_t motorSpeed;
    int32_t heading; /* direction from north (degrees) */
    int32_t dial; /* current dial position (degrees) */
    float batteryVoltage;
    int32_t batteryLevel; /* battery level - % */
    bool has_destination;
    compass_MapPoint destination;
    float direction; /* direction to the destination (degrees) */
    float distance; /* distance to destination (meters) */
    bool disableMotor;
    bool spinMotor;
    int32_t spinSpeed;
    bool calibrate; /* are we in calibration state */
    int32_t calibrateTarget; /* encoder position for calibration */
    pb_size_t currentCalibration_count;
    float currentCalibration[13]; /* current calibration values, using a repeated field to represent an array */
} compass_CompassState;

typedef struct _compass_CompassConfig {
    /* Actual configuration */
    int32_t encoderZeroDialNorth; /* where does the arrow points when encoder is 0? this correction will be applied to dial position, value depends on the encoder magnet! */
    /* Debug parameters */
    bool interpolateCalibrations; /* if false - use closest calibration, if true - interpolate calibration values (needs good calibration) */
    bool useDestination; /* if false - ignore destination and GPS, point to fixDirection on the dial */
    bool useCompass; /* if false - ignore magnetometer, set fixDirection on the dial */
    int32_t fixDirection;
    uint32_t delay; /* if delay 50, accelerometer doesn't always have time to read */
    bool ignoreHallSensor;
    bool debugHall;
    bool enableBluetooth;
    bool compensateCompassForTilt; /* flag defines compensation for tilt. Bias and matrix are applied always, because otherwise it's garbage */
    uint32_t sunriseTime; /* sunset and sunrise for UV LEDs */
    uint32_t sunsetTime;
} compass_CompassConfig;

typedef struct _compass_CalibrationData {
    float x;
    float y;
    float z;
    int32_t angle;
} compass_CalibrationData;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define compass_Coordinate_init_default          {0, 0}
#define compass_MapPoint_init_default            {0, "", 0, false, compass_Coordinate_init_default, 0}
#define compass_CompassState_init_default        {false, compass_Coordinate_init_default, 0, 0, 0, 0, 0, 0, 0, false, compass_MapPoint_init_default, 0, 0, 0, 0, 0, 0, 0, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}
#define compass_CompassConfig_init_default       {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define compass_CalibrationData_init_default     {0, 0, 0, 0}
#define compass_Coordinate_init_zero             {0, 0}
#define compass_MapPoint_init_zero               {0, "", 0, false, compass_Coordinate_init_zero, 0}
#define compass_CompassState_init_zero           {false, compass_Coordinate_init_zero, 0, 0, 0, 0, 0, 0, 0, false, compass_MapPoint_init_zero, 0, 0, 0, 0, 0, 0, 0, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}
#define compass_CompassConfig_init_zero          {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define compass_CalibrationData_init_zero        {0, 0, 0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define compass_Coordinate_latitude_tag          1
#define compass_Coordinate_longitude_tag         2
#define compass_MapPoint_id_tag                  1
#define compass_MapPoint_name_tag                2
#define compass_MapPoint_radius_tag              3
#define compass_MapPoint_coordinates_tag         4
#define compass_MapPoint_visited_tag             5
#define compass_CompassState_location_tag        1
#define compass_CompassState_havePosition_tag    2
#define compass_CompassState_closed_tag          3
#define compass_CompassState_motorSpeed_tag      4
#define compass_CompassState_heading_tag         5
#define compass_CompassState_dial_tag            6
#define compass_CompassState_batteryVoltage_tag  7
#define compass_CompassState_batteryLevel_tag    8
#define compass_CompassState_destination_tag     9
#define compass_CompassState_direction_tag       10
#define compass_CompassState_distance_tag        11
#define compass_CompassState_disableMotor_tag    12
#define compass_CompassState_spinMotor_tag       13
#define compass_CompassState_spinSpeed_tag       14
#define compass_CompassState_calibrate_tag       15
#define compass_CompassState_calibrateTarget_tag 16
#define compass_CompassState_currentCalibration_tag 17
#define compass_CompassConfig_encoderZeroDialNorth_tag 1
#define compass_CompassConfig_interpolateCalibrations_tag 2
#define compass_CompassConfig_useDestination_tag 3
#define compass_CompassConfig_useCompass_tag     4
#define compass_CompassConfig_fixDirection_tag   5
#define compass_CompassConfig_delay_tag          6
#define compass_CompassConfig_ignoreHallSensor_tag 7
#define compass_CompassConfig_debugHall_tag      8
#define compass_CompassConfig_enableBluetooth_tag 9
#define compass_CompassConfig_compensateCompassForTilt_tag 10
#define compass_CompassConfig_sunriseTime_tag    11
#define compass_CompassConfig_sunsetTime_tag     12
#define compass_CalibrationData_x_tag            1
#define compass_CalibrationData_y_tag            2
#define compass_CalibrationData_z_tag            3
#define compass_CalibrationData_angle_tag        4

/* Struct field encoding specification for nanopb */
#define compass_Coordinate_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, DOUBLE,   latitude,          1) \
X(a, STATIC,   SINGULAR, DOUBLE,   longitude,         2)
#define compass_Coordinate_CALLBACK NULL
#define compass_Coordinate_DEFAULT NULL

#define compass_MapPoint_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   id,                1) \
X(a, STATIC,   SINGULAR, STRING,   name,              2) \
X(a, STATIC,   SINGULAR, UINT32,   radius,            3) \
X(a, STATIC,   OPTIONAL, MESSAGE,  coordinates,       4) \
X(a, STATIC,   SINGULAR, BOOL,     visited,           5)
#define compass_MapPoint_CALLBACK NULL
#define compass_MapPoint_DEFAULT NULL
#define compass_MapPoint_coordinates_MSGTYPE compass_Coordinate

#define compass_CompassState_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  location,          1) \
X(a, STATIC,   SINGULAR, BOOL,     havePosition,      2) \
X(a, STATIC,   SINGULAR, BOOL,     closed,            3) \
X(a, STATIC,   SINGULAR, INT32,    motorSpeed,        4) \
X(a, STATIC,   SINGULAR, INT32,    heading,           5) \
X(a, STATIC,   SINGULAR, INT32,    dial,              6) \
X(a, STATIC,   SINGULAR, FLOAT,    batteryVoltage,    7) \
X(a, STATIC,   SINGULAR, INT32,    batteryLevel,      8) \
X(a, STATIC,   OPTIONAL, MESSAGE,  destination,       9) \
X(a, STATIC,   SINGULAR, FLOAT,    direction,        10) \
X(a, STATIC,   SINGULAR, FLOAT,    distance,         11) \
X(a, STATIC,   SINGULAR, BOOL,     disableMotor,     12) \
X(a, STATIC,   SINGULAR, BOOL,     spinMotor,        13) \
X(a, STATIC,   SINGULAR, INT32,    spinSpeed,        14) \
X(a, STATIC,   SINGULAR, BOOL,     calibrate,        15) \
X(a, STATIC,   SINGULAR, INT32,    calibrateTarget,  16) \
X(a, STATIC,   REPEATED, FLOAT,    currentCalibration,  17)
#define compass_CompassState_CALLBACK NULL
#define compass_CompassState_DEFAULT NULL
#define compass_CompassState_location_MSGTYPE compass_Coordinate
#define compass_CompassState_destination_MSGTYPE compass_MapPoint

#define compass_CompassConfig_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, INT32,    encoderZeroDialNorth,   1) \
X(a, STATIC,   SINGULAR, BOOL,     interpolateCalibrations,   2) \
X(a, STATIC,   SINGULAR, BOOL,     useDestination,    3) \
X(a, STATIC,   SINGULAR, BOOL,     useCompass,        4) \
X(a, STATIC,   SINGULAR, INT32,    fixDirection,      5) \
X(a, STATIC,   SINGULAR, UINT32,   delay,             6) \
X(a, STATIC,   SINGULAR, BOOL,     ignoreHallSensor,   7) \
X(a, STATIC,   SINGULAR, BOOL,     debugHall,         8) \
X(a, STATIC,   SINGULAR, BOOL,     enableBluetooth,   9) \
X(a, STATIC,   SINGULAR, BOOL,     compensateCompassForTilt,  10) \
X(a, STATIC,   SINGULAR, UINT32,   sunriseTime,      11) \
X(a, STATIC,   SINGULAR, UINT32,   sunsetTime,       12)
#define compass_CompassConfig_CALLBACK NULL
#define compass_CompassConfig_DEFAULT NULL

#define compass_CalibrationData_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FLOAT,    x,                 1) \
X(a, STATIC,   SINGULAR, FLOAT,    y,                 2) \
X(a, STATIC,   SINGULAR, FLOAT,    z,                 3) \
X(a, STATIC,   SINGULAR, INT32,    angle,             4)
#define compass_CalibrationData_CALLBACK NULL
#define compass_CalibrationData_DEFAULT NULL

extern const pb_msgdesc_t compass_Coordinate_msg;
extern const pb_msgdesc_t compass_MapPoint_msg;
extern const pb_msgdesc_t compass_CompassState_msg;
extern const pb_msgdesc_t compass_CompassConfig_msg;
extern const pb_msgdesc_t compass_CalibrationData_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define compass_Coordinate_fields &compass_Coordinate_msg
#define compass_MapPoint_fields &compass_MapPoint_msg
#define compass_CompassState_fields &compass_CompassState_msg
#define compass_CompassConfig_fields &compass_CompassConfig_msg
#define compass_CalibrationData_fields &compass_CalibrationData_msg

/* Maximum encoded size of messages (where known) */
#define compass_CalibrationData_size             26
#define compass_CompassConfig_size               54
#define compass_CompassState_size                485
#define compass_Coordinate_size                  18
#define compass_MapPoint_size                    292

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
