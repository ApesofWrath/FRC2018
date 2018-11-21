#include <WPILib.h>

class Task {
public:

  Task();
	virtual ~Task();

  virtual void TaskStart() {}
	virtual void TaskRun() {}
  virtual void TaskStop() {}

};
