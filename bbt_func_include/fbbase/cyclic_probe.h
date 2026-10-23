#ifndef CYCLIC_PROBE_H__
	#define CYCLIC_PROBE_H__
	#include <stdint.h>
	#include <sys/time.h>
	#include <time.h>
	#include <stdarg.h>

	typedef enum
	{
		CP_MASTERTIMER=0,  // Master timer
		CP_SENDFRAME,  // Send frame
		CP_RECVFRAME,  // Receive frame
		CP_DATAEXCH,  // Data exchange
		CP_HEARTBEAT,  // Send hearbeat
		CP_DCSYNC,  // Send DcSync
		CP_DCDIFFTIME,  // DC diff time
		CP_END
	}CYCLIC_PROBE_SIGNAL;

#ifndef USEC_PER_SEC
#define USEC_PER_SEC	(1000000)
#endif

#ifndef NSEC_PER_SEC
#define NSEC_PER_SEC (1000000000L)
#endif

#ifndef NSEC_PER_USEC
#define NSEC_PER_USEC	(1000)
#endif
	

	static inline int64_t calcdiff_us(struct timespec t1, struct timespec t2)
	{
		int64_t diff;
		diff = USEC_PER_SEC * (long long)((int) t1.tv_sec - (int) t2.tv_sec);
		diff += ((int) t1.tv_nsec - (int) t2.tv_nsec) / 1000;
		return diff;
	}

	static inline int64_t calcdiff_ns(struct timespec t1, struct timespec t2)
	{
		int64_t diff;
		diff = NSEC_PER_SEC * (int64_t)((int) t1.tv_sec - (int) t2.tv_sec);
		diff += ((int) t1.tv_nsec - (int) t2.tv_nsec);
		return diff;
	}


	int cyclic_probe_init(void);
	int cyclic_probe_register(int probeid, char* name);
	void cyclic_probe_start(int probeid, struct timespec *tp);
	void cyclic_probe_end(int probeid, struct timespec *tp);
	void cyclic_probe_interval(int probeid,  int64_t interval);
	void cyclic_probe_print(int probeid, int refresh, int warning);
	void cyclic_probe_print_all(int refresh, int warning);
	
#endif
