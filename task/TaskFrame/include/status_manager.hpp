#ifndef __TASK_STATUS_HPP__
#define __TASK_STATUS_HPP__

#include <atomic>

#include "tf_types.hpp"

#define STATUS_MANAGER_MAGIC 0x5354534D  // 'STSM'

typedef enum {
  UNDEFINED = -1,
  UNINTIATED = 0,
  UNCONFIGED = 1,
  MOTOR_OFF = 2,
  READY  = 3,
  RUNNING = 4,
  EMERGENCY_STOP = 5,
  SAFE_STOP = 6,
  ERROR = 127,
  FAIL = 128
} TASK_STATUS_TYPE;

typedef struct {
   std::atomic<int> _status_value;
   char _task_name[32];
} TASK_STATUS;

struct StatusManagerData {
    uint32_t magic = STATUS_MANAGER_MAGIC;
    uint32_t version = 1;
    TASK_STATUS task_status[MAX_TASK_NUMBER];
    int task_number = 0;
};

class CStatusManager {
private:
    StatusManagerData* data_;
    int FindTask(char* task_name);

public:
    CStatusManager() : data_(nullptr) {}
    explicit CStatusManager(StatusManagerData* data) : data_(data) {}

    static void InitData(StatusManagerData* data);

    bool NewTaskStatus(char* task_name);
    int GetTaskNumber() const { return data_ ? data_->task_number : 0; }

    bool SetStatus(char* task_name, int status_value);
    bool GetStatus(char* task_name, int& status_value);
    bool GetStatus(int task_index, int& status_value);
};

#endif
