/*
 * multithread.c
 *
 *  Created on: Feb 28, 2015
 *      Author: rdspring1
 */

#include "roneos.h"
#include "ronelib.h"

#define MAX_DELAY 100
osQueueHandle xQueue = NULL;

void task_writer(void* parameters)
{
	uint32 lastWakeTime = osTaskGetTickCount();
	int count = 0;
	char* string = (char*) parameters;

	while(TRUE)
	{
		if(count < 10)
		{
			portBASE_TYPE xStatus = osQueueSendToBack(xQueue, &string, 0);
			if(xStatus == pdPASS)
				++count;
		}
		ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD*10);
	}
}

void task_reader(void* parameters)
{
	uint32 lastWakeTime = osTaskGetTickCount();
	const portTickType xTicksToWait = 100 / portTICK_RATE_MS;
	int max = (int) parameters;
	int count = 0;

	while(TRUE)
	{
		void* string = NULL;
		portBASE_TYPE xStatus = osQueueReceive(xQueue, &string, xTicksToWait);

		if(xStatus == pdPASS)
		{
			cprintf("%s\n", (char*) string);
			++count;
		}
		else if(count < max)
		{
			cprintf("%s\n", "Failed to print from queue");
		}
		ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD*5);
	}
}

/*
int main (void)
{
	systemInit();
	systemPrintStartup();

	xQueue = osQueueCreate(20, sizeof(char*));
	if(!xQueue)
		return -1;

	osTaskCreate(task_reader, "task3", 512, 20, BEHAVIOR_TASK_PRIORITY);
	osTaskCreate(task_writer, "task1", 512, "Hippopotamus", BEHAVIOR_TASK_PRIORITY);
	osTaskCreate(task_writer, "task2", 512, "Platypus", BEHAVIOR_TASK_PRIORITY);

	osTaskStartScheduler();
	//Program should never get here
	return 0;
}
*/
