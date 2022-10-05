/**
 * rc_balance_defs.h
 *
 * Contains the settings for configuration of rc_balance.c
 */

#ifndef RC_BALANCE_CONFIG
#define RC_BALANCE_CONFIG

#define SAMPLE_RATE_HZ 100 //MPU Speed
#define SAMPLE_RATE_HZ_INNER_LOOP		100	// inner loop control speed
#define SAMPLE_RATE_HZ_OUTER_LOOP		20	// outer loop control speed

// Structural properties of eduMiP
#define BOARD_MOUNT_ANGLE	0.49 // increase if mip tends to roll forward
#define GEARBOX			35.577
#define ENCODER_RES		60
#define WHEEL_RADIUS_M		0.034
#define TRACK_WIDTH_M		0.035
#define V_NOMINAL		7.4

// inner loop controller 100hz
#define D1_GAIN			1
#define D1_ORDER		1
#define D1_NUM			{-2.769, 2.618}
#define D1_DEN			{1, -.999}
#define D1_NUM_LEN		2
#define D1_DEN_LEN		2
#define D1_SATURATION_TIMEOUT	10


// outer loop controller 20hz
#define D2_GAIN			1
#define	D2_ORDER		1
#define D2_NUM			{.0588, -.0556}
#define D2_DEN			{1, -.600}
#define D2_NUM_LEN		2
#define D2_DEN_LEN		2
#define THETA_REF_MAX		0.33

// electrical hookups
#define MOTOR_CHANNEL_L		3
#define MOTOR_CHANNEL_R		2
#define MOTOR_POLARITY_L	-1
#define MOTOR_POLARITY_R	1
#define ENCODER_CHANNEL_L	2
#define ENCODER_CHANNEL_R	3
#define ENCODER_POLARITY_L	-1
#define ENCODER_POLARITY_R	1

// Thread Loop Rates
#define BATTERY_CHECK_HZ	5
#define SETPOINT_MANAGER_HZ	100
#define PRINTF_HZ		50

// other
#define TIP_ANGLE		0.85
#define START_ANGLE		0.3
#define START_DELAY		0.4
#define PICKUP_DETECTION_TIME	0.6
#define ENABLE_POSITION_HOLD	1
#define SOFT_START_SEC		0.7

#endif	// endif RC_BALANCE_CONFIG
