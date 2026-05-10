#pragma once
#include "freertos/FreeRTOS.h"

typedef void *SemaphoreHandle_t;

SemaphoreHandle_t xSemaphoreCreateMutex(void);
SemaphoreHandle_t xSemaphoreCreateRecursiveMutex(void);
BaseType_t xSemaphoreTake(SemaphoreHandle_t semaphore, TickType_t ticks);
BaseType_t xSemaphoreGive(SemaphoreHandle_t semaphore);
BaseType_t xSemaphoreTakeRecursive(SemaphoreHandle_t semaphore, TickType_t ticks);
BaseType_t xSemaphoreGiveRecursive(SemaphoreHandle_t semaphore);
void vSemaphoreDelete(SemaphoreHandle_t semaphore);
