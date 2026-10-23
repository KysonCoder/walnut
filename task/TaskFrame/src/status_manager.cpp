#include <cstring>
#include "status_manager.hpp"
#include "utilbase/logger.h"

void CStatusManager::InitData(StatusManagerData* data) {
    data->magic = STATUS_MANAGER_MAGIC;
    data->version = 1;
    data->task_number = 0;
    for (int i = 0; i < MAX_TASK_NUMBER; i++) {
        strcpy(data->task_status[i]._task_name, "");
        data->task_status[i]._status_value.store(0);
    }
}

int CStatusManager::FindTask(char* task_name) {
    if (!data_) return -1;
    for (int i = 0; i < MAX_TASK_NUMBER; i++) {
        if (strcmp(task_name, data_->task_status[i]._task_name) == 0) {
            return i;
        }
    }
    return -1;
}

bool CStatusManager::NewTaskStatus(char* task_name) {
    if (!data_) return false;
    int index = FindTask(task_name);
    if (index != -1) {
        Log_D("Task %s already exist", task_name);
        return false;
    }
    strcpy(data_->task_status[data_->task_number]._task_name, task_name);
    data_->task_number++;
    return true;
}

bool CStatusManager::SetStatus(char* task_name, int status_value) {
    if (!data_) return false;
    int index = FindTask(task_name);
    if (index == -1) {
        Log_D("Task %s not found", task_name);
        return false;
    }
    data_->task_status[index]._status_value = status_value;
    return true;
}

bool CStatusManager::GetStatus(char* task_name, int& status_value) {
    if (!data_) return false;
    int index = FindTask(task_name);
    if (index == -1) {
        Log_D("Task %s not found", task_name);
        return false;
    }
    status_value = data_->task_status[index]._status_value.load();
    return true;
}

bool CStatusManager::GetStatus(int task_index, int& status_value) {
    if (!data_ || task_index >= data_->task_number || task_index < 0) {
        Log_D("Task index %d out of range", task_index);
        return false;
    }
    status_value = data_->task_status[task_index]._status_value.load();
    return true;
}
