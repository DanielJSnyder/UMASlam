#ifndef __SLAM_CONSTANTS_HPP__
#define __SLAM_CONSTANTS_HPP__

#define GPS_CHANNEL "SENSOR_GPS"
#define FOG_CHANNEL "SENSOR_FOG"
#define LASER_SCAN_CHANNEL "SENSOR_LASER"
#define STATE_CHANNEL "STATE_CHANNEL"
#define SERVO_CHANNEL "SENSOR_LASER_SERVO"
#define SLAM_STATE_CHANNEL "SLAM_STATE"
#define SLAM_POINT_CLOUD_CHANNEL "SLAM_POINT_CLOUD"
#define SLAM_PARTICLE_CHANNEL "SLAM_PARTICLES"

//profiling constants
#define NUM_PROFILED_SCANS 30

//general constants (FULL SLAM)
#define NUM_ONLY_MAP_SCANS 1
#define MAX_X 75
#define MIN_X -75
#define MAX_Y 75
#define MIN_Y -75
#define SQUARE_SIZE 0.5

//localization constants
#define HIT_LIKELIHOOD_INC_VALUE 1.0
#define HIT_THRESHOLD 175
#define NUM_AVERAGE_PARTICLES 5
#define NUM_OUTLIERS_TO_REMOVE 2
#define NUM_PARTICLES 2000
const static double PERCENT_PREDICTION_PARTICLES = 0.50;

//Data sheet values 1.5
#define DEFAULT_GPS_SIGMA 1.0

//Data sheet value 0.5
#define DEFAULT_FOG_SIGMA 0.5*M_PI/180.0

//Just guessing
const static double X_PREDICTION_SIGMA = 0.25;
const static double Y_PREDICTION_SIGMA = 0.25;

//mapping constants
#define INITIAL_MAP_VALUE 128
#define FULL_SQUARE_INC 5.0
static const double EMPTY_SQUARE_INC = -0.025;

//point cloud constants
#define DEFAULT_MISS_RANGE 15
static const double LIDAR_HEIGHT = 0.15;

//fake compass stuff
#define USE_FAKE_COMPASS 1
#define ORIGIN_DIST_BEFORE_REINITIALIZATION 15

//Drawing stuff
#define NUM_POSES_TO_DRAW 50

#endif
