#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>
#include "sensors/proximity.h"
#include <camera/po8030.h>
#include "spi_comm.h"
#include <obstacle.h>
#include <audio_processing.h>
#include <fft.h>
#include <process_image.h>
#include <arm_math.h>
#include "leds.h"
//uncomment to send the FFTs results from the real microphones
#define SEND_FROM_MIC

bool initialised, direction_acquired, direction_changed = FALSE;

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void status(bool status){
	initialised=status;
}
void direction(bool status){
	direction_acquired=status;
}
void change(bool status){
	direction_changed=status;
}
void SendUint8ToComputer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}



int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();

    //inits the motors
    motors_init();
    //configuration camera
    dcmi_start();
	po8030_start();
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    proximity_start();

	process_image_start();




/*#ifdef SEND_FROM_MIC
    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);
#endif  /* SEND_FROM_MIC */

    /* Infinite loop. */
    while (1) {
    	 chThdSleepMilliseconds(100);

    	/*if(initialised==FALSE){
#ifdef SEND_FROM_MIC
        //waits until a result must be sent to the computer
        wait_send_to_computer();

#endif  /* SEND_FROM_MIC */
  //  	}
    	//else{
    		//chThdSleepMilliseconds(100);
    	  /*  if(!direction_acquired||direction_changed){
    	    	 arm_copy_f32(get_audio_buffer_ptr(LEFT_OUTPUT), micLeft, 2*FFT_SIZE);
    	    	 arm_copy_f32(get_audio_buffer_ptr(RIGHT_OUTPUT), micRight, 2*FFT_SIZE);
    	    	 arm_copy_f32(get_audio_buffer_ptr(FRONT_OUTPUT), micFront, 2*FFT_SIZE);
    	    	 arm_copy_f32(get_audio_buffer_ptr(BACK_OUTPUT), micBack, 2*FFT_SIZE);

    	    	 audio_detection(micLeft, micRight, micFront, micBack, micLeft_mag, micFront_mag);
    	    }
    	   // else{
    	    	obstacle_start();
    	    //}
    	 }*/
    	obstacle_start();

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
