#include <stdio.h>
#include "headers/BRO_spam_fists.h"
#include "headers/BRO_spam_client.h"
#include <stdbool.h>
#define END_COMM 6455
#define RUN_TIME 2000
#define CONTROL_TARGET 2
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
    U32 connect_status = 0; 
    RefPacket in_packet;
    memset((void*)&in_packet, 0, sizeof(RefPacket)); 
    DataPacket out_packet;
    U32 time_rec_data;
    U32 time_rec_control;
    time_rec_control = systick_get_ms();
    time_rec_data = systick_get_ms();
    float target_speed = 0;
    float actual_target =0; 
    float et[4] = {0, 0, 0,0};
    float ut[4] = {0,0,0,0};
    float speedt[3] = {0,0,0};
    float step; 
    int16_t pres_count = 0, prev_count = 0; 
    pres_count = nxt_motor_get_count(NXT_PORT_B);
    prev_count = pres_count;
    connect_status = ecrobot_read_bt(&in_packet,0, sizeof(RefPacket));
    ecrobot_send_bt(&out_packet,0, sizeof(DataPacket));
    connect_status = ecrobot_read_bt(&in_packet,0, sizeof(RefPacket));
    target_speed = CONTROL_TARGET;
    actual_target = target_speed/1;
    uint16_t j = 0;
    time_rec_data = systick_get_ms();
    while(j<RUN_TIME){
        pres_count = nxt_motor_get_count(NXT_PORT_B);
        speedt[2] = speedt[1];
        speedt[1] = speedt[0];
        if(pres_count != prev_count){
        	speedt[0] = ((pres_count - prev_count)*(3.14/180)*1000)/((systick_get_ms() - time_rec_control));
            prev_count = pres_count;
            time_rec_control = systick_get_ms();
        }
        et[0] = (actual_target - speedt[0]); 
        ut[0] = (((413.1956*et[0]) - (799.608*et[1]) + (387.195*et[2]))*20 + (800*ut[1]) - (385*ut[2]))/415;
       step = ut[0];
        	if(step > 100)
        		nxt_motor_set_speed(NXT_PORT_B, 100, 1);
        	else if(step < -100)
		    	nxt_motor_set_speed(NXT_PORT_B, -100, 1);
       		else
				nxt_motor_set_speed(NXT_PORT_B, step, 1);
    memset(&out_packet, 0, sizeof(DataPacket));
    j++; 
    out_packet.count = nxt_motor_get_count(NXT_PORT_B); 
    out_packet.delta = (systick_get_ms() - time_rec_data);
    time_rec_data = systick_get_ms();
    ecrobot_send_bt(&out_packet,0, sizeof(DataPacket));
    if (connect_status == -1)
    	ecrobot_status_monitor("Failed to receive");
    ut[3] = ut[2];
    ut[2] = ut[1]; 
    ut[1] = ut[0];
    et[3] = et[2];
    et[2] = et[1]; 
    et[1] = et[0];
    
    systick_wait_ms(5);
    }
    ecrobot_device_terminate();
    TerminateTask();
}

TASK(DisplayTask)
{
    ecrobot_status_monitor("BROFist Client");
    TerminateTask();
}
