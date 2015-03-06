/*
 * multithread.c
 *
 *  Created on: Feb 28, 2015
 *      Author: rdspring1
 */

#include "roneos.h"
#include "ronelib.h"

#define MAX_DELAY 100
osSemaphoreHandle xMutex = NULL;

void serial_send_string_mutex(const char* string)
{
	osSemaphoreTake(xMutex, MAX_DELAY);
	{
		cprintf("%s\n", string);
	}
	osSemaphoreGive(xMutex);

}

void task1(void* parameters)
{
	int count = 0;

	while(TRUE)
	{
		if(count < 10)
		{
			serial_send_string_mutex((char*) parameters);
			++count;
		}
		ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
	}
}

void task2(void* parameters)
{
	int count = 0;

	while(TRUE)
	{
		if(count < 10)
		{
			serial_send_string_mutex((char*) parameters);
			++count;
		}
		ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
	}
}

/*
int main (void)
{
	systemInit();
	systemPrintStartup();

	xMutex = osSemaphoreCreateMutex();

	osTaskCreate(task1, "task1", 4096, "Hippopotamus", BEHAVIOR_TASK_PRIORITY);
	osTaskCreate(task2, "task2", 4096, "Platypus", BEHAVIOR_TASK_PRIORITY);

	osTaskStartScheduler();
	//Program should never get here
	return 0;
}
*/
