#include "roneos.h"
#include "ronelib.h"

//
void self_stabilizing_tree(void* parameters)
{
	// TODO Send/Receive Radio/IR Messages
	// TODO Receive Neighbor IDs and Bearing
	// Resources -rprintf, Game->robotstuff / joystickrobot
	uint32 lastWakeTime = osTaskGetTickCount();

	while(TRUE)
	{
		// Get Neighbors
		// Initial - Assume this robot is the root
		// Find the neighbor that is closest to this robot with the lowest id
		// Send unicast message to parent to measure hop count
		osTaskDelayUntil(&lastWakeTime, BEHAVIOR_TASK_PERIOD*10);
	}
}

int main (void)
{
	systemInit();
	systemPrintStartup();
	buttonsInit();

	behaviorSystemInit(buttonColors, 4096);

	osTaskStartScheduler();
	//Program should never get here
	return 0;
}
