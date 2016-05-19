#include <stdio.h>
#include "headers/BRO_spam_fists.h"
#include "headers/BRO_spam_client.h"
#include <stdbool.h>
#define RUN_TIME 600
#define RADIUS 0.028
#define LINEAR_VELOCITY 0.392
#define LEFT_MOTOR NXT_PORT_A
#define RIGHT_MOTOR NXT_PORT_C
#define SPEED_VARIETY 4
/*--------------------------------------------------------------------------*/
/* OSEK declarations                                                        */
/*--------------------------------------------------------------------------*/
DeclareCounter(SysTimerCnt);
DeclareResource(lcd);
DeclareTask(BRO_Comm);
DeclareTask(PID_Controller);
DeclareTask(DisplayTask);

engines_t engines;
U8 flag = 1;
float linear_velocity[SPEED_VARIETY] = {0.14, 0.196, 0.252, 0.308};
/*typedef struct spam_data {
	float rev_count[10]; 
    float time_stamp[10];
}spam_data;

typedef struct outdata {
    //uint8_t buffsize;
    spam_data packets;
}outdata; */


#if 0
engines_t spam_motor_start() {
    engines_t ret = {
        {.port = NXT_PORT_A, .speed_control_type = NOT_USING, .speed_ref = 0},
        {.port = NXT_PORT_B, .speed_control_type = NOT_USING, .speed_ref = 0},
        {.port = NXT_PORT_C, .speed_control_type = NOT_USING, .speed_ref = 0}
    };
    
    return ret;
};
#endif

/*--------------------------------------------------------------------------*/
/* LEJOS OSEK hooks                                                         */
/*--------------------------------------------------------------------------*/
void ecrobot_device_initialize()
{
    ecrobot_init_bt_slave("1234");
    
    memset(&engines, 0, sizeof(engines_t));

    engines.first.port = NXT_PORT_A;
    engines.first.speed_control_type = NOT_USING;
    engines.first.speed_ref = 0;

    engines.second.port = NXT_PORT_B;
    engines.second.speed_control_type = NOT_USING;
    engines.second.speed_ref = 0;

    engines.third.port = NXT_PORT_C;
    engines.third.speed_control_type = NOT_USING;
    engines.third.speed_ref = 0;

    if (CONN_SONAR) {
        ecrobot_init_sonar_sensor(SONAR_PORT);
    };
    if (CONN_LIGHT) {
        ecrobot_set_light_sensor_active(LIGHT_PORT);
    };
}


void ecrobot_device_terminate()
{
  
    memset(&engines, 0, sizeof(engines_t));

    nxt_motor_set_speed(NXT_PORT_A, 0, 1);
    nxt_motor_set_speed(NXT_PORT_B, 0, 1);
    nxt_motor_set_speed(NXT_PORT_C, 0, 1);
        
    ecrobot_set_light_sensor_inactive(LIGHT_PORT);
    ecrobot_term_sonar_sensor(SONAR_PORT);

    bt_reset();

    ecrobot_term_bt_connection();
}

/*--------------------------------------------------------------------------*/
/* Function to be invoked from a category 2 interrupt                       */
/*--------------------------------------------------------------------------*/
void user_1ms_isr_type2(void)
{
    StatusType ercd;

    /*
     *  Increment OSEK Alarm System Timer Count
    */
    ercd = SignalCounter( SysTimerCnt );
    if ( ercd != E_OK ) {
        ShutdownOS( ercd );
    }
}

/*--------------------------------------------------------------------------*/
/* Task information:                                                        */
/* -----------------                                                        */
/* Name    : PID_Controller                                                 */
/* Priority: 3                                                              */
/* Typ     : EXTENDED TASK                                                  */
/* Schedule: preemptive                                                     */
/*--------------------------------------------------------------------------*/
TASK(PID_Controller)
{
    
    /*  We will now update the powers for each motor.
     *  If a motor is set as RAW_POWER then the first "powers" value is used
     *  directly for the speed update.
     *  Oblviously it will be possible for the users to not install a motor,
     *  so we won't do anything with the ports on which there is nothing.
     */
    
    if (engines.first.speed_control_type != NOT_USING) {
        //  Now we will set the powers using the data inside the motor's
        //  structure.
        nxt_motor_set_speed(engines.first.port, engines.first.powers[0], 1);
    };
    
    //  Doing the same thing for the second motor
    if (engines.second.speed_control_type != NOT_USING) {
        nxt_motor_set_speed(engines.second.port, engines.second.powers[0], 1);
    };
    
    //  And, guess what? We are doing it even for the third motor.
    if (engines.third.speed_control_type != NOT_USING) {
        nxt_motor_set_speed(engines.third.port, engines.third.powers[0], 1);
    };    

    TerminateTask();
}

TASK(BRO_Comm)
{
    U32 ml_time_rec_control, mr_time_rec_control;
    ml_time_rec_control = systick_get_ms();
    mr_time_rec_control = systick_get_ms();
    int diff_power = 0;
    float ml_speed = 0, mr_speed = 0;
    float ml_step = 0, mr_step = 0; 
    float target_linear_velocity;
    S32 ml_pres_count = 0, ml_prev_count = 0;
    S32 mr_pres_count = 0, mr_prev_count = 0; 
    uint16_t j = 0, k = 0;
    while(1){
    	for(k=0; k < SPEED_VARIETY; k++) {
            display_clear(0);
        	display_goto_xy(0,0);
        
        	display_int(k,3);
            
    		target_linear_velocity = linear_velocity[k];
        	ml_time_rec_control = systick_get_ms();
    		mr_time_rec_control = systick_get_ms();
        	float ml_et[3] = {0, 0, 0};
    		float mr_et[3] = {0, 0, 0};
    		float ml_ut[3] = {0, 0, 0};
    		float mr_ut[3] = {0, 0, 0};
        	ml_speed = 0;
        	mr_speed = 0;
        	ml_step = 0;  
        	mr_step = 0; 
        	ml_pres_count = nxt_motor_get_count(LEFT_MOTOR);
    		mr_pres_count = nxt_motor_get_count(RIGHT_MOTOR);
    		ml_prev_count = ml_pres_count;
    		mr_prev_count = mr_pres_count;
            j=0; // reset while loop control variable
            nxt_motor_set_speed(LEFT_MOTOR, 0, 1); // apply brake for the robot
            nxt_motor_set_speed(RIGHT_MOTOR, 0, 1);
            systick_wait_ms(2000); // Wait for 2 seconds before taking over next speed
    		while(j<RUN_TIME){
            	// Track Left Motor 
        		ml_pres_count = nxt_motor_get_count(LEFT_MOTOR);
        		if(ml_pres_count != ml_prev_count){
        			ml_speed = ((ml_pres_count - ml_prev_count)*(3.14/180)*1000)/((systick_get_ms() - ml_time_rec_control));
            		ml_prev_count = ml_pres_count;
            		ml_time_rec_control = systick_get_ms();
        		}
        		ml_et[0] = (target_linear_velocity/RADIUS) - ml_speed; 
                        ml_ut[0] = (((413.1956*ml_et[0]) - (799.608*ml_et[1]) + (387.195*ml_et[2]))*20 + (800*ml_ut[1]) - (385*ml_ut[2]))/415;
       			ml_step = ml_ut[0];
            	// Track Right Motor
            	mr_pres_count = nxt_motor_get_count(RIGHT_MOTOR);
        		if(mr_pres_count != mr_prev_count){
        			mr_speed = ((mr_pres_count - mr_prev_count)*(3.14/180)*1000)/((systick_get_ms() - mr_time_rec_control));
            		mr_prev_count = mr_pres_count;
            		mr_time_rec_control = systick_get_ms();
        		}
        		mr_et[0] = (target_linear_velocity/RADIUS) - mr_speed; 
        		mr_ut[0] = (((420.25*mr_et[0]) - (799.5*mr_et[1]) + (380.25*mr_et[2]))*20 + (800*mr_ut[1]) - (385*mr_ut[2]))/415;
                        ml_ut[0] = (((413.1956*mr_et[0]) - (799.608*mr_et[1]) + (387.195*mr_et[2]))*20 + (800*mr_ut[1]) - (385*mr_ut[2]))/415;
       			mr_step = mr_ut[0];
            	// Fine tune and balance
            	diff_power = (ml_pres_count - mr_pres_count)*1; // Gain to be varied for better tuning on the motor 
            	if(diff_power > 0){
            		ml_step = ml_step - diff_power;
                	mr_step = mr_step + diff_power; 
            	} 
            	else if (diff_power < 0){
                	ml_step = ml_step - diff_power;
                	mr_step = mr_step + diff_power;
            	}
            	// Apply correct Left Motor
            	if(ml_step > 100)
        			nxt_motor_set_speed(LEFT_MOTOR, 100, 1);
        		else if(ml_step < -100)
		    		nxt_motor_set_speed(LEFT_MOTOR, -100, 1);
       			else
					nxt_motor_set_speed(LEFT_MOTOR, ml_step, 1);
    			ml_ut[2] = ml_ut[1]; 
    			ml_ut[1] = ml_ut[0];
    			ml_et[2] = ml_et[1]; 
    			ml_et[1] = ml_et[0];
            	// Apply correction Right Motor
            	if(mr_step > 100)
        			nxt_motor_set_speed(RIGHT_MOTOR, 100, 1);
        		else if(mr_step < -100)
		    		nxt_motor_set_speed(RIGHT_MOTOR, -100, 1);
       			else
					nxt_motor_set_speed(RIGHT_MOTOR, mr_step, 1);  
    			mr_ut[2] = mr_ut[1]; 
    			mr_ut[1] = mr_ut[0];
    			mr_et[2] = mr_et[1]; 
    			mr_et[1] = mr_et[0];
            	j++;
    			systick_wait_ms(5);
    		}
    	}
	}
    ecrobot_device_terminate();
    TerminateTask();
}

TASK(DisplayTask)
{
    ecrobot_status_monitor("BROFist Client");
    TerminateTask();
}
