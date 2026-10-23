/** \file os.h
 * \brief Operating system abstract
 *
 * Usage : API to access to operating system resources.
 *
 *
 *
 * 
 *
 * (c) AT  2022.02.02
 */

#ifndef __OS_H_
	#define __OS_H_

 #include <stdbool.h>
 
#ifdef __cplusplus
extern "C"
{
#endif


#include <semaphore.h>
#include <pthread.h>

/*Infinite wait*/
#define OS_WAIT_INFINITE	-1

/*Os semphore type*/
typedef void * OS_SEM;
/*Os mutex type*/
typedef void * OS_MUTEX;
/*Os event type*/
typedef void * OS_EVENT;
/*Os flag type*/
typedef void * OS_FLAG;


/*Os semphore structure*/
struct os_sem
{	
	char name[32];
	int  type;
	sem_t handle;
};
/*Os mutex structure*/
struct os_mutex
{
	char name[32];
	int  type;
	pthread_mutex_t handle;
};
/*Os event structure*/
struct os_event 
{
	char name[32];
	int  type;
	pthread_cond_t handle;
      pthread_mutex_t evt_mutx;
      volatile bool evt_status;
      bool evt_manual_reset;	
};

/*Os flag structure*/
struct os_flag 
{
	char name[32];
	int  type;
	long handle;
};


 /* Os version */
 const char *os_version (void);

/*Create realtime task*/
int os_task_create_rt(void *thandle, int stacksize, void *func, void *param, char* taskname, int priority, int cpucore);
/*Create non-realtime task*/
int os_task_create(void* thandle, int stacksize, void* func, void* param, char* taskname, int cpucore);
/*Sleep with some ms*/
int os_sleep (int time_ms);
/*Get os current tick*/
unsigned int os_get_tick(void);

/** Create os smphore
 *
 * @param[in]  name, semphore name, only for threads now, not check same anme
 * @param[in]  initial_count, semphore initial count
 * @
 * @
 * @return semphore handle
 */
OS_SEM os_sem_create ( char* name, int initial_count);

/** Create os mutex
 *
 * @param[in]  name, mutex name, only for threads now, not check same anme
 * @
 * @
 * @return mutex handle
 */
OS_MUTEX os_mutex_create (char* name);

/** Create os event
 *
 * @param[in]  name, event name, only for threads now, not check same anme
 * @param[in]  initial_state, event initial state
 * @param[in]  manual_reset, the event whether need manual reset 
 * @
 * @
 * @return event handle
 */
OS_MUTEX os_event_create (char* name, bool initial_state, bool manual_reset);

/** Create os flag
 *
 * @param[in]  name, flag name
 * @
 * @
 * @return flag handle
 */
OS_FLAG os_flag_create (char* name);


/** Wait os smphore
 *
 * @param[in]  sem_handle, os semphore handle
 * @param[in]  timeout, wait some time
 * @
 * @
 * @return true indicates success to get semphore, others failed
 */
bool  os_sem_wait( OS_SEM sem_handle, int timeout);

/** Post os smphore
 *
 * @param[in]  sem_handle, semphore handle
 * @
 * @
 * @return true indicates success to release semphore, others failed
 */
bool  os_sem_post( OS_SEM sem_handle);

/** Destroy os smphore
 *
 * @param[in]  sem_handle, semphore handle
 * @
 * @
 * @return true indicates success to destroy semphore, others failed
 */
bool  os_sem_destroy( OS_SEM sem_handle);

/** Wait os mutex
 *
 * @param[in]  mutex_handle, os mutex handle
 * @param[in]  timeout, wait some time
 * @
 * @
 * @return true indicates success to get mutex, others failed
 */
bool  os_mutex_wait( OS_MUTEX mutex_handle, int timeout);

/** Post os mutex
 *
 * @param[in]  mutex_handle, os mutex handle
 * @
 * @
 * @return true indicates success to release mutex, others failed
 */
bool  os_mutex_post( OS_MUTEX mutex_handle);

/** Destroy os mutex
 *
 * @param[in]  mutex_handle, os mutex handle
 * @
 * @
 * @return true indicates success to destroy mutex, others failed
 */
bool  os_mutex_destroy( OS_MUTEX mutex_handle);

/** Wait os event
 *
 * @param[in]  event_handle, os event handle
 * @param[in]  timeout, wait some time
 * @
 * @
 * @return true indicates success to get event, others failed
 */
bool  os_event_wait( OS_EVENT event_handle, int timeout);

/** Set os event
 *
 * @param[in]  event_handle, os event handle
 * @
 * @
 * @return true indicates success to set event, others failed
 */
bool  os_event_set( OS_EVENT event_handle);

/** Reset os event
 *
 * @param[in]  event_handle, os event handle
 * @
 * @
 * @return true indicates success to reset event, others failed
 */
bool  os_event_reset( OS_EVENT event_handle);

/** Destroy os event
 *
 * @param[in]  event_handle, os event handle
 * @
 * @
 * @return true indicates success to destroy event, others failed
 */
bool  os_event_destroy( OS_EVENT event_handle);

/** Set os flag
 *
 * @param[in]  flag_handle, os flag handle
 * @
 * @
 * @return true indicates success to set flag, others failed
 */
bool  os_flag_set( OS_FLAG flag_handle);

/** Clear os flag
 *
 * @param[in]  flag_handle, os flag handle
 * @
 * @
 * @return true indicates success to clear flag, others failed
 */
bool  os_flag_clear( OS_FLAG flag_handle);

/** Destroy os flag
 *
 * @param[in]  flag_handle, os flag handle
 * @
 * @
 * @return true indicates success to destroy flag, others failed
 */
bool  os_flag_destroy( OS_FLAG flag_handle);

#ifdef __cplusplus
}
#endif

#endif // __OS_H_
