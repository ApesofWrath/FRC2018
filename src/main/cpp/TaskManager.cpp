#include "TaskManager.h"
#include "tasks/Task.h"

TaskManager::TaskManager(
	void
): m_numTasks(0)
, m_thread_low()
, m_thread_high()
, m_thread_low_running(false)
, m_thread_high_running(false)
, m_mutex(PTHREAD_MUTEX_INITIALIZER)
{
}

TaskManager::~TaskManager() {
	Stop();
}

void TaskManager::Start(void) {
	pthread_mutex_lock(&m_mutex)
	if(!m_thread_low_running){
		m_thread_low_running = true;
		pthread_create(&m_thread_low, NULL, TaskLowFreqAll, this);
	}
	if(!m_thread_high_running){
		m_thread_high_running = true;
		pthread_create(&m_thread_high, NULL, TaskHighFreqAll, this);
	}
}

bool TaskManager::RegisterTask(const char *taskName, Task *task, uint32_t flags) {
	bool success = false;
	int index;

	index = this->FindTask(task);
	if (index != -1) {
		//If the task is already registered, just add to the existing flags
		m_taskFlags[index] |= flags;
		success = true;
	}
	else if (m_numTasks < MAX_NUM_TASKS) {
		strncpy(m_taskNames[m_numTasks], taskName, MAX_TASK_NAME_LEN);
		m_tasks[m_numTasks] = task;
		m_taskFlags[m_numTasks] = flags;
		m_numTasks++;
		success = true;
	}

	fprintf(stderr, "Task %s registered to %p with result %d (1 is go0d)\n",
	taskName, this, success);

	return success;
}

bool TaskManager::UnregisterTask(Task *task) {
	bool success = false;
	int taskIndex, j;

	taskIndex = this->FindTask(task);
	if (taskIndex != -1) {
		//Shift tasks to replace the removed one
		for (j = taskIndex + 1; j < m_numTasks; j++) {
			strcpy(m_taskNames[j - 1], m_taskNames[j]);
			m_tasks[j - 1] = m_tasks[j];
			m_taskFlags[j - 1] = m_taskFlags[j];
		}

		//clear out the last task in the list
		m_numTasks--;
		m_taskNames[m_numTasks][0] = '\0';
		m_tasks[m_numTasks] = NULL;
		m_taskFlags[m_numTasks] = 0;

		success = true;
	}

	return success;
}

void TaskManager::TaskStartModeAll(RobotMode mode) {
	uint64_t startTime, endTime;

	for (int i = 0; i < m_numTasks; i++) {
		if (m_taskFlags[i] & TASK_START_MODE) {
			startTime = GetUsecTime();
			m_tasks[i]->TaskStartMode(mode);
			endTime = GetUsecTime();

			if (ENABLE_PROFILING) {
				printf("TaskStartMode(%d) for %s took %llu us\n",
				mode, m_taskNames[i], endTime - startTime);
			}
		}
	}
}

void TaskManager::TaskStopModeAll(RobotMode mode) {
	uint64_t startTime, endTime;

	//stop tasks in the reverse order they were started in
	for (int i = m_numTasks - 1; i >= 0; i--) {
		if (m_taskFlags[i] & TASK_STOP_MODE) {
			startTime = GetUsecTime();
			m_tasks[i]->TaskStopMode(mode);
			endTime = GetUsecTime();

			if (ENABLE_PROFILING) {
				printf("TaskStopMode(%d) for %s took %llu us\n",
				mode, m_taskNames[i], endTime - startTime);
			}
		}
	}
}

void TaskManager::TaskLowFreqAll(RobotMode mode) {
	uint64_t startTime, endTime;

	for (int i = 0; i < m_numTasks; i++) {
		if (m_taskFlags[i] & TASK_PERIODIC) {
			startTime = GetUsecTime();
			m_tasks[i]->TaskPeriodic(mode);
			endTime = GetUsecTime();

			if (ENABLE_PROFILING) {
				printf("TaskPeriodicAll(%d) for %s took %llu us\n",
				mode, m_taskNames[i], endTime - startTime);
			}
		}
	}
}

void TaskManager::TaskHighFreqAll(RobotMode mode) {
	uint64_t startTime, endTime;

	for (int i = 0; i < m_numTasks; i++) {
		if (m_taskFlags[i] & TASK_PERIODIC) {
			startTime = GetUsecTime();
			m_tasks[i]->TaskPeriodic(mode);
			endTime = GetUsecTime();

			if (ENABLE_PROFILING) {
				printf("TaskPeriodicAll(%d) for %s took %llu us\n",
				mode, m_taskNames[i], endTime - startTime);
			}
		}
	}
}

int TaskManager::FindTask(Task *task) {
	int index = -1;

	for (int i = 0; i < m_numTasks; i++) {
		if (m_tasks[i] == task) {
			index = i;
			break;
		}
	}

	return index;
}

}
