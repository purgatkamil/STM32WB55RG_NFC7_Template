#ifndef APP_FREERTOS_CONFIG_H
#define APP_FREERTOS_CONFIG_H

#include <assert.h>

#include <stm32wbxx.h>


#define configUSE_PREEMPTION                      1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION   0
#define configUSE_16_BIT_TICKS                    0
#define configUSE_TICKLESS_IDLE                   1

#define configUSE_TASK_NOTIFICATIONS              1
#define configUSE_MUTEXES                         1
#define configUSE_RECURSIVE_MUTEXES               1
#define configUSE_COUNTING_SEMAPHORES             1
#define configUSE_QUEUE_SETS                      0
#define configUSE_CO_ROUTINES                     0
#define configUSE_TIMERS                          1

/*KP*/
#define configTOTAL_HEAP_SIZE                    ((size_t)12288)
#define configMAX_TASK_NAME_LEN                  ( 16 )
#define configUSE_TRACE_FACILITY                 1

#define configUSE_IDLE_HOOK                       0
#define configUSE_TICK_HOOK                       1
#define configUSE_MALLOC_FAILED_HOOK              0
#define configUSE_DAEMON_TASK_STARTUP_HOOK        0


#define configCPU_CLOCK_HZ                        SystemCoreClock
#define configTICK_RATE_HZ                        ((TickType_t)1000)
#define configMAX_PRIORITIES                      ( 56 )
#define configMINIMAL_STACK_SIZE                  128
#define configENABLE_BACKWARD_COMPATIBILITY       0
#define configTASK_NOTIFICATION_ARRAY_ENTRIES     2

#define configUSE_NEWLIB_REENTRANT          1

//#ifdef DEBUG
//#define configCHECK_FOR_STACK_OVERFLOW            2
//#endif /* DEBUG */

#define configKERNEL_INTERRUPT_PRIORITY           (15 << (8 - __NVIC_PRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY      (5 << (8 - __NVIC_PRIO_BITS))

#define configSUPPORT_STATIC_ALLOCATION           1
#define configSUPPORT_DYNAMIC_ALLOCATION          1

#ifndef NDEBUG
#define configASSERT(X)                           assert(X)
#endif /* NDEBUG */


#define INCLUDE_vTaskPrioritySet                  1
#define INCLUDE_uxTaskPriorityGet                 1
#define INCLUDE_vTaskDelete                       1
#define INCLUDE_vTaskSuspend                      1
#define INCLUDE_xResumeFromISR                    1
//#define INCLUDE_xTaskDelayUntil                   1
#define INCLUDE_vTaskDelay                        1
#define INCLUDE_xTaskGetSchedulerState            1
#define INCLUDE_xTaskGetCurrentTaskHandle         1
#define INCLUDE_uxTaskGetStackHighWaterMark       1
#define INCLUDE_xTaskGetIdleTaskHandle            0
#define INCLUDE_eTaskGetState                     1
#define INCLUDE_xEventGroupSetBitFromISR          0
#define INCLUDE_xTimerPendFunctionCall            1
#define INCLUDE_xTaskAbortDelay                   0
#define INCLUDE_xTaskGetHandle                    0
#define INCLUDE_xTaskResumeFromISR                0

/* KP*/
#define INCLUDE_xQueueGetMutexHolder              1
#define INCLUDE_vTaskDelayUntil                   1

#define configTIMER_TASK_PRIORITY                 4
#define configTIMER_QUEUE_LENGTH                  10
#define configTIMER_TASK_STACK_DEPTH              configMINIMAL_STACK_SIZE

#define USE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION 1

#define xPortPendSVHandler                        PendSV_Handler
#define vPortSVCHandler                           SVC_Handler




#endif /* APP_FREERTOS_CONFIG_H */
