#pragma once

#include "FreeRTOS.h"
#include "task.h"

#include <stdlib.h>


TaskHandle_t pxGetTaskFromStack( TaskHandle_t pxTask, StackType_t* pxStack ) {
    extern TCB_t * volatile pxCurrentTCB;
    if (! pxTask) {
        pxTask = pxCurrentTCB;
    }
    if (pxTask && pxStack >= pxTask->pxStack && pxStack <= pxTask->pxEndOfStack) {
        return pxTask;
    }

    return NULL;
}
