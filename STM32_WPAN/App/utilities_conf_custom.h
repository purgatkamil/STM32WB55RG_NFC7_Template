/*
 * utilities_conf_custom.h
 *
 *  Created on: May 23, 2024
 *      Author: KP
 */

#ifndef APP_UTILITIES_CONF_CUSTOM_H_
#define APP_UTILITIES_CONF_CUSTOM_H_

#define UTIL_SEQ_CHECK_IDLE( ) 																										\
				do{																													\
					UTIL_SEQ_ENTER_CRITICAL_SECTION( );																				\
					int can_enter_idle_mode = (0U == ((local_taskset & local_taskmask & SuperMask) | (local_evtset & EvtWaited)));	\
					UTIL_SEQ_EXIT_CRITICAL_SECTION( );																				\
					if(can_enter_idle_mode)																							\
					{																												\
						extern void UTIL_SEQ_Own_Idle( );																				\
						UTIL_SEQ_Own_Idle( );																							\
					}																												\
				  }while(0)																											\

  #define UTIL_SEQ_ENTER_CRITICAL_SECTION_IDLE( )    do { UTIL_SEQ_CHECK_IDLE(); if (0) {
  #define UTIL_SEQ_EXIT_CRITICAL_SECTION_IDLE( )     } } while(0)

#endif /* APP_UTILITIES_CONF_CUSTOM_H_ */
