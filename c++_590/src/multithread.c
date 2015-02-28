/*
 * multithread.c
 *
 *  Created on: Feb 28, 2015
 *      Author: rdspring1
 */

#include "roneos.h"
#include "ronelib.h"

void task1(void* parameters)
{
	const char* pcTaskName = "Hippopotamus";

	for( ; ; )
	{
		cprintf("%s\n", pcTaskName);
		ledsSetPattern(LED_BLUE, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
	}
}

void task2(void* parameters)
{
	const char* pcTaskName = "Platypus";

	for( ; ; )
	{
		cprintf("%s\n", pcTaskName);
		ledsSetPattern(LED_GREEN, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
	}
}

void task3(void* parameters)
{
	uint32 lastWakeTime = osTaskGetTickCount();
	const char* pcTaskName = "task periodic";

	for( ; ; )
	{
		cprintf("%s\n", pcTaskName);
		ledsSetPattern(LED_RED, LED_PATTERN_CIRCLE, LED_BRIGHTNESS_MED, LED_RATE_MED);
		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD*10);
	}
}

int main (void)
{
	systemInit();
	systemPrintStartup();

	osTaskCreate(task3, "task3", 4096, NULL, BEHAVIOR_TASK_PRIORITY*4);
	osTaskCreate(task2, "task2", 4096, NULL, BEHAVIOR_TASK_PRIORITY);
	osTaskCreate(task1, "task1", 4096, NULL, BEHAVIOR_TASK_PRIORITY*2);

	osTaskStartScheduler();
	//Program should never get here
	return 0;
}
