// project_config.h
#pragma once

// FreeRTOS Core
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// Task Prioritäten
#define TASK_PRIO_TORQUE   1  // Höchste Priorität
#define TASK_PRIO_BUTTON   1
#define TASK_PRIO_CAN      1
#define TASK_PRIO_STATUS   1  
#define TASK_PRIO_BLE      1
#define TASK_PRIO_DISPLAY  1 // Niedrigste Priorität

// Stack Größen
#define TASK_STACK_SIZE 2048
#define STACK_SIZE_TORQUE   2048
#define STACK_SIZE_BUTTON   2048
#define STACK_SIZE_DISPLAY  2048


