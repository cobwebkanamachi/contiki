#include "contiki.h"
#include "lib/sensors.h"
#include "dev/button-sensor.h"
#include <AppHardwareApi.h>
#include "dev/button-sensor.h"
#include "Button.h"
#include "dev/leds.h"

#define BUTTONS_ENABELED_CONF 1

#ifdef BUTTONS_ENABELED_CONF
#define BUTTONS_ENABELED BUTTONS_ENABELED_CONF
#else
#define BUTTONS_ENABELED 0
#endif

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

const struct sensors_sensor button_sensor;
static uint32_t volatile irq_val=0,
                             val=0;
#define BUTTON0_DIO (BUTTON_0_PIN)
#define BUTTONS_DIO BUTTON0_DIO

#define BUTTON0_INT E_AHI_DIO8_INT

PROCESS(debounce_process, "button debounce");
#define DEBOUNCE_TIME (CLOCK_SECOND/50)
/*---------------------------------------------------------------------------*/
void button_ISR(uint32 u32DeviceId,	uint32 u32ItemBitmap)
{
  if (BUTTONS_ENABELED) {
//  	if(! (u32DeviceId & E_AHI_DEVICE_SYSCTRL && u32ItemBitmap & BUTTON0_INT)) {
//  		return;
//  	}
  	/* announce the change */
  	sensors_changed(&button_sensor);
  	/* disable this IRQ while the button stops oscillating */
		vAHI_DioInterruptEnable(0, BUTTON0_DIO);
		//irq_val = u8ButtonReadRfd();
		/* let the debounce process know about this to enable it again */
		process_poll(&debounce_process);
	  leds_on(LEDS_ALL);
		PRINTF("button_ISR\n");
  }
}
/*---------------------------------------------------------------------------*/
void
button_press(void)
{
  sensors_changed(&button_sensor);
}
/*---------------------------------------------------------------------------*/
static int
value(int type)
{
	switch (type & BUTTONS_ENABELED) {
	case BUTTON_0_MASK:
		return val & BUTTON_0_MASK;
//	case BUTTON_1_MASK:
//		return val & BUTTON_1_MASK;
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
static int
configure(int type, int value)
{
	if (BUTTONS_ENABELED) {
		switch (type) {
		case SENSORS_HW_INIT:
//			//vButtonInitRfd();
//			vAHI_DioSetDirection(BUTTONS_DIO, 0x00);
//			vAHI_SysCtrlRegisterCallback(button_ISR);
//			PRINTF("SENSORS_HW_INIT\n");
//			//vAHI_DioInterruptEdge(uint32 u32Rising,	uint32 u32Falling);
//			//val = u8ButtonReadRfd();
//	    val = (u32AHI_DioReadInput() & BUTTONS_DIO);
//	  	process_start(&debounce_process, NULL);
//			return 1;
		case SENSORS_ACTIVE:
			PRINTF("SENSORS_ACTIVE %u\n", value);
			if (!value) {
				//deactivate
				vAHI_DioInterruptEnable(0, BUTTON0_DIO);
			} else {
				vButtonInitRfd();
				val = u8ButtonReadRfd();
//				vAHI_DioSetDirection(BUTTONS_DIO, 0x00);
//		    val = (u32AHI_DioReadInput() & BUTTONS_DIO);

				PRINTF("SENSORS_HW_INIT\n");

				vAHI_SysCtrlRegisterCallback(button_ISR);
				//activate
				vAHI_DioInterruptEnable(BUTTON0_DIO, 0);
				vAHI_DioInterruptEdge(0, BUTTON0_DIO);

				process_start(&debounce_process, NULL);
			}
			break;
		case SENSORS_READY:
			PRINTF("SENSORS_READY\n");
			return 1;
		default:
			return 0;
		}
	}
	return 1;
}
/*---------------------------------------------------------------------------*/
static int
status(int type)
{
  return u8ButtonReadRfd() & type;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(debounce_process, ev, data)
{
  static struct etimer et;

	if (BUTTONS_ENABELED) {
		PROCESS_BEGIN();
		PRINTF("debounce_process begin\n");

		while (1) {
			PROCESS_YIELD_UNTIL(ev==PROCESS_EVENT_POLL || ev==PROCESS_EVENT_TIMER);

			if(ev==PROCESS_EVENT_TIMER) {
//				if (val != irq_val) {
//					PRINTF("val,irq_val=%d,%d\n",val,irq_val);
//					val = irq_val;
//					sensors_changed(&button_sensor);
//				}
				PRINTF("debounce_process reactivate IRQ\n");
				//reactivate IRQ
				vAHI_DioInterruptEnable(BUTTON0_DIO, 0);
			  leds_off(LEDS_ALL);
			} else if(ev==PROCESS_EVENT_POLL) {
				etimer_set(&et, DEBOUNCE_TIME);
				PRINTF("debounce_process set timer to reactivate IRQ\n");
			}
		}
		PROCESS_END();
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(button_sensor, BUTTON_SENSOR,
	       value, configure, status);
