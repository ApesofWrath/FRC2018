/*
* TaskMgr.h
*
*  Created on: Sep 5, 2015
*      Author: Andrew
*
* Modified version of 973's 2017 robot code
*/

//#pragma once

#include "globals.h"
#include "pthread.h"
#include "tasks/Task.h"
#include "states/State.h"

#define MAX_NUM_TASKS			32
#define MAX_TASK_NAME_LEN		31

// static constexpr double FREQUENCY_LOW_HZ 10.0; //won't build
// static constexpr double FREQUENCY_HIGH_HZ 100.0;
//
// static constexpr double PERIOD_LOW_MS = 1000.0 / FREQUENCY_LOW_HZ;
// static constexpr double PERIOD_HIGH_MS = 1000.0 / FREQUENCY_HIGH_HZ;

using namespace frc;

class TaskManager {

public:

	TaskManager(State &robotState);

	virtual ~TaskManager();

	/**
	* Register Task object.  If the task is already registered, the flags
	* 		passed here are added to those flags for which the task was already
	* 		registered.  If the task registry is already full, return  false.
	*
	* @param taskName Specifies the name of the task being registered (for
	* 		debug purposes)
	* @param task Specifies the task to be registered
	* @param flags Specifies which callbacks should be called for the given
	* 		task
	*
	* @return Returns true if the task was successfully registered.  False
	* 		otherwise.
	*/
	bool RegisterTask(const char *taskName, Task *task, uint32_t flags);


	void Start(void);


	void Stop(void);




	/**
	* Function to unregister Task
	*
	* @param task Specifies the Task to unregister
	*
	* @return Retuns true if Task successfully unregistered, false
	* 		otherwise (for example if the task was not registered).
	*/
	bool UnregisterTask(Task *task);

protected:
	/**
	* Calls the TaskStartMode method of all Task objects
	*
	* @param mode Specifies robot operation mode
	*/
	void TaskStartModeAll(RobotMode mode);

	/**
	* Calls the TaskStopMode method of all Task objects
	*
	* @param mode Specifies robot operation mode
	*/
	void TaskStopModeAll(RobotMode mode);

	/**
	* Calls the TaskLowFreq method of all Task objects at 10Hz frequency
	*
	* @param mode Specifies robot operation mode
	*/
	void TaskLowFreqAll(RobotMode mode);

	/**
	* Calls the TaskHighFreq method of all Task objects at 100Hz frequency
	*
	* @param mode Specifies robot operation mode
	*/
	void TaskHighFreqAll(RobotMode mode);

private:
	/**
	* Returns the index of the given task int he register tasks list
	*
	* @param Task to look up
	*
	* @return If found, the task index is returned.  Otherwise it returns -1
	*/
	int FindTask(CoopTask *task);

	int			m_numTasks;
	char	    m_taskNames[MAX_NUM_TASKS][MAX_TASK_NAME_LEN + 1];
	Task   *m_tasks[MAX_NUM_TASKS];

	pthread_t m_thread_low;
	pthread_t m_thread_high;
	pthread_mutex_t m_mutex;
	bool m_thread_low_running;
	bool m_thread_high_running;
};

}
