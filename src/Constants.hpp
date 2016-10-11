#ifndef __SLAM_CONSTANTS_HPP__
#define __SLAM_CONSTANTS_HPP__

#define GPS_CHANNEL "SENSOR_GPS"
#define FOG_CHANNEL "SENSOR_FOG"
#define LASER_SCAN_CHANNEL "SENSOR_LASER"
#define STATE_CHANNEL "STATE_CHANNEL"
#define SERVO_CHANNEL "SENSOR_LASER_SERVO"
#define SLAM_STATE_CHANNEL "SLAM_STATE"
#define SLAM_POINT_CLOUD_CHANNEL "SLAM_POINT_CLOUD"

//profiling constants
#define NUM_PROFILED_SCANS 30

//general constants (FULL SLAM)
#define NUM_ONLY_MAP_SCANS 5
#define MAX_X 50
#define MIN_X -50
#define MAX_Y 50
#define MIN_Y -50
#define SQUARE_SIZE 0.5
#define NUM_PARTICLES 1000

//localization constants
#define HIT_LIKELIHOOD_INC_VALUE 1.0
#define HIT_THRESHOLD 150
//Data sheet values 1.5
#define DEFAULT_GPS_SIGMA 1.5

//Data sheet value 0.5
#define DEFAULT_FOG_SIGMA 0.5

//mapping constants
#define FULL_SQUARE_INC 1.0
#define EMPTY_SQUARE_INC (-1.0)

#endif
