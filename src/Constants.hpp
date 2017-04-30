#ifndef __SLAM_CONSTANTS_HPP__
#define __SLAM_CONSTANTS_HPP__

#include <cmath> //for M_PI

#define GPS_CHANNEL "SENSOR_GPS"
#define FOG_CHANNEL "SENSOR_FOG"
#define LASER_SCAN_CHANNEL "SENSOR_LASER"
#define STATE_CHANNEL "STATE_CHANNEL"
#define SERVO_CHANNEL "SENSOR_LASER_SERVO"
#define SLAM_STATE_CHANNEL "SLAM_STATE"
#define SLAM_MAP_CHANNEL "SLAM_MAP"
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
const static int HIT_THRESHOLD = 160;
const static int MISS_THRESHOLD = 75;
const static int NUM_AVERAGE_PARTICLES = 1;
const static int NUM_OUTLIERS_TO_REMOVE = 0;
const static int NUM_PARTICLES = 1000;

//Lidar related localization coefficients
const static double HIT_LIKELIHOOD_INC_VALUE = 1.0;
const static double MISS_LIKELIHOOD_DEC_VALUE = -5.0;
const static int MINIMUM_LIDAR_HITS_TO_WEIGHT = 100;

//GPS related localization coefficients
//unit is meters
//Data sheet values 1.5
const static double DEFAULT_GPS_SIGMA = 1.5;

//FOG related localization coefficients
//unit is radians
//Data sheet value 0.5
const static double DEFAULT_FOG_SIGMA = 0.5*M_PI/180.0;

//Localization coefficients related to predicting particles forward with gps
const static double PERCENT_PREDICTION_PARTICLES = 0.50;
const static double X_PREDICTION_SIGMA = 0.1;
const static double Y_PREDICTION_SIGMA = 0.1;

//Localization constants that relate the relative beliefs in the various sensors
const static double LASER_LIKELIHOOD_COEFFICIENT = 0.75;
const static double GPS_LIKELIHOOD_COEFFICIENT = 2.5;
const static double FOG_LIKELIHOOD_COEFFICIENT = 1.0;

//mapping constants
#define INITIAL_MAP_VALUE 128
#define FULL_SQUARE_INC 5.0
static const double EMPTY_SQUARE_INC = -0.025;
static const double LIDAR_MAP_RANGE_DEG = 60.0;

//point cloud constants
#define DEFAULT_MISS_RANGE 15
static const double LIDAR_HEIGHT = 0.15;

//fake compass stuff
#define USE_FAKE_COMPASS 1
#define ORIGIN_DIST_BEFORE_REINITIALIZATION 5

//Drawing stuff
#define NUM_POSES_TO_DRAW 50

#endif
