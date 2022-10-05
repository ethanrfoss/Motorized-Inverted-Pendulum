/**
* @example rc_balance
*
* Reference solution for balancing EduMiP
**/

#include <stdio.h>
#include <unistd.h> // for isatty()
#include <stdlib.h> // for strtof()
#include <math.h>   // for M_PI
#include <robotcontrol.h>

#include "rc_balance_defs_ethan.h"

/**
 * ARMED or DISARMED to indicate if the controller is running
 */
typedef enum arm_state_t{
	ARMED,
	DISARMED
}arm_state_t;

/**
 * Feedback controller setpoint written to by setpoint_manager and read by the
 * controller.
 */
typedef struct setpoint_t{
	arm_state_t arm_state;	///< see arm_state_t declaration
	double theta;		///< body lean angle (rad)
	double phi;		///< wheel position (rad)
}setpoint_t;

/**
 * This is the system state written to by the balance controller.
 */
typedef struct core_state_t{
	double wheelAngleR;	///< wheel rotation relative to body
	double wheelAngleL;
	double theta;		///< body angle radians
	double phi;		///< average wheel angle in global frame
	double vBatt;		///< battery voltage
	double d1_u;		///< output of balance controller D1 to motors
	double d2_u;		///< output of position controller D2 (theta_ref)
	double mot_drive;	///< u compensated for battery voltage
} core_state_t;

// possible modes, user selected with command line arguments
typedef enum m_input_mode_t{
	NONE,
	DSM,
	STDIN
} m_input_mode_t;


static void __print_usage(void);
static void* __inner_loop(void* ptr);	///< background thread
static void* __outer_loop(void* ptr);	///< background thread
static void* __battery_checker(void* ptr);	///< background thread
static void __update_measurements(void);	///< background thread
static void* __printf_loop(void* ptr);		///< background thread
static int __zero_out_controller(void);
static int __disarm_controller(void);
static int __arm_controller(void);
static int __wait_for_starting_condition(void);
static void __on_pause_press(void);


// global variables
// Must be declared static so they get zero initalized
static core_state_t cstate;
static setpoint_t setpoint;
static rc_filter_t D1 = RC_FILTER_INITIALIZER;
static rc_filter_t D2 = RC_FILTER_INITIALIZER;
static rc_mpu_data_t mpu_data;
static m_input_mode_t m_input_mode = DSM;

/*
 * printed if some invalid argument was given
 */
static void __print_usage(void)
{
	printf("\n");
	printf("-i {dsm|stdin|none}     specify input\n");
	printf("-q                      Don't print diagnostic info\n");
	printf("-h                      print this help message\n");
	printf("\n");
}

/**
 * Initialize the filters, mpu, threads, & wait until shut down
 *
 * @return     0 on success, -1 on failure
 */
int main(int argc, char *argv[])
{
	int c;
	pthread_t battery_thread = 0;
	pthread_t inner_loop_thread = 0;
	pthread_t outer_loop_thread = 0;
	pthread_t printf_thread = 0;
	bool adc_ok = true;
	bool quiet = false;

	// parse arguments
	opterr = 0;
	while ((c = getopt(argc, argv, "i:qh")) != -1){
		switch (c){
		case 'i': // input option
			if(!strcmp("dsm", optarg)) {
				m_input_mode = DSM;
			} else if(!strcmp("stdin", optarg)) {
				m_input_mode = STDIN;
			} else if(!strcmp("none", optarg)){
				m_input_mode = NONE;
			} else {
				__print_usage();
				return -1;
			}
			break;
		case 'q':
			quiet = true;
			break;
		case 'h':
			__print_usage();
			return -1;
			break;
		default:
			__print_usage();
			return -1;
			break;
		}
	}

	// make sure another instance isn't running
	// if return value is -3 then a background process is running with
	// higher privaledges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
	if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
	if(rc_enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}

	// initialize buttons
	if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to initialize pause button\n");
		return -1;
	}
	if(rc_button_init(RC_BTN_PIN_MODE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to initialize mode button\n");
		return -1;
	}

	// Assign functions to be called when button events occur
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE,__on_pause_press,NULL);

	// initialize enocders
	if(rc_encoder_eqep_init()==-1){
		fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
		return -1;
	}

	// initialize motors
	if(rc_motor_init()==-1){
		fprintf(stderr,"ERROR: failed to initialize motors\n");
		return -1;
	}
	rc_motor_standby(1); // start with motors in standby

	// start dsm listener
	if(m_input_mode == DSM){
		if(rc_dsm_init()==-1){
			fprintf(stderr,"failed to start initialize DSM\n");
			return -1;
		}
	}

	// initialize adc
	if(rc_adc_init()==-1){
		fprintf(stderr, "failed to initialize adc\n");
		adc_ok = false;
	}

	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	printf("\nPress and release MODE button to toggle DSM drive mode\n");
	printf("Press and release PAUSE button to pause/start the motors\n");
	printf("hold pause button down for 2 seconds to exit\n");

	if(rc_led_set(RC_LED_GREEN, 0)==-1){
		fprintf(stderr, "ERROR in rc_balance, failed to set RC_LED_GREEN\n");
		return -1;
	}
	if(rc_led_set(RC_LED_RED, 1)==-1){
		fprintf(stderr, "ERROR in rc_balance, failed to set RC_LED_RED\n");
		return -1;
	}

	// set up mpu configuration
	rc_mpu_config_t mpu_config = rc_mpu_default_config();
	mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	mpu_config.orient = ORIENTATION_Y_UP;

	// if gyro isn't calibrated, run the calibration routine
	if(!rc_mpu_is_gyro_calibrated()){
		printf("Gyro not calibrated, automatically starting calibration routine\n");
		printf("Let your MiP sit still on a firm surface\n");
		rc_mpu_calibrate_gyro_routine(mpu_config);
	}

	// make sure setpoint starts at normal values
	setpoint.arm_state = DISARMED;

	// set up D1 Theta controller
	double D1_num[] = D1_NUM;
	double D1_den[] = D1_DEN;
	if(rc_filter_alloc_from_arrays(&D1, SAMPLE_RATE_HZ_INNER_LOOP, D1_num, D1_NUM_LEN, D1_den, D1_DEN_LEN)){
		fprintf(stderr,"ERROR in rc_balance, failed to make filter D1\n");
		return -1;
	}
	D1.gain = D1_GAIN;
	rc_filter_enable_saturation(&D1, -1.0, 1.0);
	rc_filter_enable_soft_start(&D1, SOFT_START_SEC);

	// set up D2 Phi controller
	double D2_num[] = D2_NUM;
	double D2_den[] = D2_DEN;
	if(rc_filter_alloc_from_arrays(&D2, SAMPLE_RATE_HZ_OUTER_LOOP, D2_num, D2_NUM_LEN, D2_den, D2_DEN_LEN)){
		fprintf(stderr,"ERROR in rc_balance, failed to make filter D2\n");
		return -1;
	}
	D2.gain = D2_GAIN;
	rc_filter_enable_saturation(&D2, -THETA_REF_MAX, THETA_REF_MAX);
	rc_filter_enable_soft_start(&D2, SOFT_START_SEC);

	printf("Inner Loop controller D1:\n");
	rc_filter_print(D1);
	printf("\nOuter Loop controller D2:\n");
	rc_filter_print(D2);

	// start a thread to slowly sample battery
	if (adc_ok) {
		if(rc_pthread_create(&battery_thread, __battery_checker, (void*) NULL, SCHED_OTHER, 0)){
			fprintf(stderr, "failed to start battery thread\n");
			return -1;
		}
	}
	else { // If we can't get the battery voltage
		cstate.vBatt = V_NOMINAL; // Set to a nominal value
	}

	// wait for the battery thread to make the first read
	while(cstate.vBatt<1.0 && rc_get_state()!=EXITING) rc_usleep(10000);

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	if(isatty(fileno(stdout)) && (quiet == false)){
		if(rc_pthread_create(&printf_thread, __printf_loop, (void*) NULL, SCHED_OTHER, 0)){
			fprintf(stderr, "failed to start printf thread\n");
			return -1;
		}
	}

	if(rc_pthread_create(&inner_loop_thread, __inner_loop, (void*) NULL, SCHED_OTHER, 0)){
		fprintf(stderr, "failed to start inner loop thread\n");
		return -1;
	}

	if(rc_pthread_create(&outer_loop_thread, __outer_loop, (void*) NULL, SCHED_OTHER, 0)){
		fprintf(stderr, "failed to start outer loop thread\n");
		return -1;
	}

	// start mpu
	if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
		fprintf(stderr,"ERROR: can't talk to IMU, all hope is lost\n");
		rc_led_blink(RC_LED_RED, 5, 5);
		return -1;
	}

	// this should be the last step in initialization
	// to make sure other setup functions don't interfere
	rc_mpu_set_dmp_callback(&__update_measurements);

	// start in the RUNNING state, pressing the pause button will swap to
	// the PAUSED state then back again.
	printf("\nHold your MIP upright to begin balancing\n");
	rc_set_state(RUNNING);

	// chill until something exits the program
	rc_set_state(RUNNING);
	while(rc_get_state()!=EXITING){
		rc_usleep(200000);
	}

	// join threads
	if (inner_loop_thread) rc_pthread_timed_join(inner_loop_thread, NULL, 1.5);
	if (outer_loop_thread) rc_pthread_timed_join(outer_loop_thread, NULL, 1.5);
	if (battery_thread) rc_pthread_timed_join(battery_thread, NULL, 1.5);
	if (printf_thread) rc_pthread_timed_join(printf_thread, NULL, 1.5);

	// cleanup
	rc_filter_free(&D1);
	rc_filter_free(&D2);
	rc_mpu_power_off();
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);
	rc_led_cleanup();
	rc_encoder_eqep_cleanup();
	rc_button_cleanup();	// stop button handlers
	rc_remove_pid_file();	// remove pid file LAST
	return 0;
}


/**
 * discrete-time inner loop controller operated off mpu interrupt Called at
 * SAMPLE_RATE_HZ
 */
void* __inner_loop(__attribute__ ((unused)) void* ptr)
{
	
	// wait for mpu to settle
	__disarm_controller();
	rc_usleep(2500000);
	rc_set_state(RUNNING);
	rc_led_set(RC_LED_RED,0);
	rc_led_set(RC_LED_GREEN,1);

	static int inner_saturation_counter = 0;
	while(rc_get_state()!=EXITING){
		/************************************************************
		* INNER LOOP ANGLE Theta controller D1
		* Input to D1 is theta error (setpoint-state). Then scale the
		* output u to compensate for changing battery voltage.
		*************************************************************/

		rc_usleep(1000000 / SAMPLE_RATE_HZ_INNER_LOOP);

		if(setpoint.arm_state == DISARMED){
			if(__wait_for_starting_condition()==0){
				__zero_out_controller();
				__arm_controller();
			}
			else continue;
		}

		D1.gain = D1_GAIN * V_NOMINAL/cstate.vBatt;
		cstate.d1_u = rc_filter_march(&D1,(setpoint.theta-cstate.theta));

		/*************************************************************
		* Check if the inner loop saturated. If it saturates for over
		* a second disarm the controller to prevent stalling motors.
		*************************************************************/
		if(setpoint.arm_state==ARMED){
			if(fabs(cstate.d1_u)>0.95) inner_saturation_counter++;
			else inner_saturation_counter = 0;
			// if saturate for a second, disarm for safety
			if(inner_saturation_counter > (SAMPLE_RATE_HZ_INNER_LOOP*D1_SATURATION_TIMEOUT)){
				printf("inner loop controller saturated\n");
				__disarm_controller();
				inner_saturation_counter = 0;
				return NULL;
			}
		}

		/**********************************************************
		* Send signal to motors
		* add D1 balance control u and D3 steering control also
		* multiply by polarity to make sure direction is correct.
		***********************************************************/
		rc_motor_set(MOTOR_CHANNEL_L, MOTOR_POLARITY_L * cstate.d1_u);
		rc_motor_set(MOTOR_CHANNEL_R, MOTOR_POLARITY_R * cstate.d1_u);
	}
	return NULL;
}

/**
 * discrete-time outer loop controller operated off mpu interrupt Called at
 * SAMPLE_RATE_HZ
 */
void* __outer_loop(__attribute__ ((unused)) void* ptr)
{
	
	// wait for mpu to settle
	__disarm_controller();
	rc_usleep(2500000);
	rc_set_state(RUNNING);
	rc_led_set(RC_LED_RED,0);
	rc_led_set(RC_LED_GREEN,1);

	while(rc_get_state()!=EXITING){
		
		
		rc_usleep(1000000 / SAMPLE_RATE_HZ_OUTER_LOOP);;
		if(setpoint.arm_state == DISARMED){
			if(__wait_for_starting_condition()==0){
				__zero_out_controller();
				__arm_controller();
			}
			else continue;
		}

		/************************************************************
		* OUTER LOOP PHI controller D2
		* Move the position setpoint based on phi_dot.
		* Input to the controller is phi error (setpoint-state).
		*************************************************************/
		cstate.d2_u = rc_filter_march(&D2,setpoint.phi-cstate.phi);
		setpoint.theta = cstate.d2_u;
	}

	return NULL;

}

static void __update_measurements(void)
{

	/******************************************************************
	* STATE_ESTIMATION
	* read sensors and compute the state when either ARMED or DISARMED
	******************************************************************/
	// angle theta is positive in the direction of forward tip around X axis
	cstate.theta = mpu_data.dmp_TaitBryan[TB_PITCH_X] + BOARD_MOUNT_ANGLE;
	
	// collect encoder positions, right wheel is reversed
	cstate.wheelAngleR = (rc_encoder_eqep_read(ENCODER_CHANNEL_R) * 2.0 * M_PI) \
			/(ENCODER_POLARITY_R * GEARBOX * ENCODER_RES);
	cstate.wheelAngleL = (rc_encoder_eqep_read(ENCODER_CHANNEL_L) * 2.0 * M_PI) \
			/(ENCODER_POLARITY_L * GEARBOX * ENCODER_RES);

	// Phi is average wheel rotation also add theta body angle to get absolute
	// wheel position in global frame since encoders are attached to the body
	cstate.phi = ((cstate.wheelAngleL+cstate.wheelAngleR)/2) + cstate.theta;

	/*************************************************************
	* check for various exit conditions AFTER state estimate
	***************************************************************/
	if(rc_get_state()==EXITING){
		rc_motor_set(0,0.0);
		return;
	}
	// if controller is still ARMED while state is PAUSED, disarm it
	if(rc_get_state()!=RUNNING && setpoint.arm_state==ARMED){
		__disarm_controller();
		return;
	}
	// exit if the controller is disarmed
	if(setpoint.arm_state==DISARMED){
		return;
	}

	// check for a tipover
	if(fabs(cstate.theta) > TIP_ANGLE){
		__disarm_controller();
		printf("tip detected \n");
		return;
	}

	return;
}

/**
 * Clear the controller's memory and zero out setpoints.
 *
 * @return     { description_of_the_return_value }
 */
static int __zero_out_controller(void)
{
	rc_filter_reset(&D1);
	rc_filter_reset(&D2);
	setpoint.theta = 0.0;
	setpoint.phi   = 0.0;
	rc_motor_set(0,0.0);
	return 0;
}

/**
 * disable motors & set the setpoint.core_mode to DISARMED
 *
 * @return     { description_of_the_return_value }
 */
static int __disarm_controller(void)
{
	rc_motor_standby(1);
	rc_motor_free_spin(0);
	setpoint.arm_state = DISARMED;
	return 0;
}

/**
 * zero out the controller & encoders. Enable motors & arm the controller.
 *
 * @return     0 on success, -1 on failure
 */
static int __arm_controller(void)
{
	__zero_out_controller();
	rc_encoder_eqep_write(ENCODER_CHANNEL_L,0);
	rc_encoder_eqep_write(ENCODER_CHANNEL_R,0);
	// prefill_filter_inputs(&D1,cstate.theta);
	rc_motor_standby(0);
	setpoint.arm_state = ARMED;
	setpoint.theta = 0;
	setpoint.phi = 0;
	return 0;
}

/**
 * Wait for MiP to be held upright long enough to begin. Returns
 *
 * @return     0 if successful, -1 if the wait process was interrupted by pause
 *             button or shutdown signal.
 */
static int __wait_for_starting_condition(void)
{
	int checks = 0;
	const int check_hz = 20;	// check 20 times per second
	int checks_needed = round(START_DELAY*check_hz);
	int wait_us = 1000000/check_hz;

	// wait for MiP to be tipped back or forward first
	// exit if state becomes paused or exiting
	while(rc_get_state()==RUNNING){
		// if within range, start counting
		if(fabs(cstate.theta) > START_ANGLE) checks++;
		// fell out of range, restart counter
		else checks = 0;
		// waited long enough, return
		if(checks >= checks_needed) break;
		rc_usleep(wait_us);
	}
	// now wait for MiP to be upright
	checks = 0;
	// exit if state becomes paused or exiting
	while(rc_get_state()==RUNNING){
		// if within range, start counting
		if(fabs(cstate.theta) < START_ANGLE) checks++;
		// fell out of range, restart counter
		else checks = 0;
		// waited long enough, return
		if(checks >= checks_needed) return 0;
		rc_usleep(wait_us);
	}
	return -1;
}

/**
 * Slow loop checking battery voltage. Also changes the D1 saturation limit
 * since that is dependent on the battery voltage.
 *
 * @return     nothing, NULL poitner
 */
static void* __battery_checker(__attribute__ ((unused)) void* ptr)
{
	double new_v;
	while(rc_get_state()!=EXITING){
		new_v = rc_adc_batt();
		// if the value doesn't make sense, use nominal voltage
		if (new_v>9.0 || new_v<5.0) new_v = V_NOMINAL;
		cstate.vBatt = new_v;
		rc_usleep(1000000 / BATTERY_CHECK_HZ);
	}
	return NULL;
}

/**
 * prints diagnostics to console this only gets started if executing from
 * terminal
 *
 * @return     nothing, NULL pointer
 */
static void* __printf_loop(__attribute__ ((unused)) void* ptr)
{
	rc_state_t last_rc_state, new_rc_state; // keep track of last state
	last_rc_state = rc_get_state();
	while(rc_get_state()!=EXITING){
		new_rc_state = rc_get_state();
		// check if this is the first time since being paused
		if(new_rc_state==RUNNING && last_rc_state!=RUNNING){
			printf("\nRUNNING: Hold upright to balance.\n");
			printf("    θ    |");
			printf("  θ_ref  |");
			printf("    φ    |");
			printf("  φ_ref  |");
			printf("  D1_u   |");
			printf("  vBatt  |");
			printf("arm_state|");
			printf("\n");
		}
		else if(new_rc_state==PAUSED && last_rc_state!=PAUSED){
			printf("\nPAUSED: press pause again to start.\n");
		}
		last_rc_state = new_rc_state;

		// decide what to print or exit
		if(new_rc_state == RUNNING){
			printf("\r");
			printf("%7.3f  |", cstate.theta);
			printf("%7.3f  |", setpoint.theta);
			printf("%7.3f  |", cstate.phi);
			printf("%7.3f  |", setpoint.phi);
			printf("%7.3f  |", cstate.d1_u);
			printf("%7.3f  |", cstate.vBatt);

			if(setpoint.arm_state == ARMED) printf("  ARMED  |");
			else printf("DISARMED |");

			fflush(stdout);
		}
		rc_usleep(1000000 / PRINTF_HZ);
	}
	return NULL;
}

/**
 * Disarm the controller and set system state to paused. If the user holds the
 * pause button for 2 seconds, exit cleanly
 */
static void __on_pause_press(void)
{
	int i=0;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds

	switch(rc_get_state()){
	// pause if running
	case EXITING:
		return;
	case RUNNING:
		rc_set_state(PAUSED);
		__disarm_controller();
		rc_led_set(RC_LED_RED,1);
		rc_led_set(RC_LED_GREEN,0);
		break;
	case PAUSED:
		rc_set_state(RUNNING);
		__disarm_controller();
		rc_led_set(RC_LED_GREEN,1);
		rc_led_set(RC_LED_RED,0);
		break;
	default:
		break;
	}

	// now wait to see if the user want to shut down the program
	while(i<samples){
		rc_usleep(us_wait/samples);
		if(rc_button_get_state(RC_BTN_PIN_PAUSE)==RC_BTN_STATE_RELEASED){
			return; //user let go before time-out
		}
		i++;
	}
	printf("long press detected, shutting down\n");
	//user held the button down long enough, blink and exit cleanly
	rc_led_blink(RC_LED_RED,5,2);
	rc_set_state(EXITING);
	return;
}
