#ifndef _CONFIG_H_
#define _CONFIG_H_

//#define		MASTER
#define		SLAVE

//#define SNIFFER

#define MAX_NODE_NUMBER					8									// max number of support node in the network
#define MAX_NODE_NUMBER_DIV8			(uint8)((MAX_NODE_NUMBER - 1) / 8 + 1)

////////////////////////////////////////////////////////////////////////////////////////////////////
//
//			Standard Defines
//
////////////////////////////////////////////////////////////////////////////////////////////////////
#if !defined(FALSE)
#define FALSE 0
#endif

#if !defined(TRUE)
#define TRUE (!FALSE)
#endif

#if !defined(NULL)
#define NULL (void *)0
#endif

typedef signed   char		int8;
typedef unsigned char		uint8;

typedef signed   short		int16;
typedef unsigned short		uint16;

typedef signed   long		int32;
typedef unsigned long		uint32;


extern const uint8 LOCAL_NODE_ID;
extern const uint8 APPLICATION_DATA_LENGTH;
extern uint8 demo_data[];
extern const uint8 JOIN_INTERVAL;
extern const uint8 DATA_INTERVAL;
extern const uint8 DISCOVERY_INTERVAL;
extern const uint8 TOPOLOGY_INTERVAL;
extern const uint8 RESPONSE_INTERVAL;
extern const uint8 RESPONSE_TIME_PERIOD;
extern const uint8 SLAVE_BROADCAST_TIME_PERIOD;
extern const uint8 MAX_DATA_REPORT_TIMES;
extern const uint8 MAX_JOIN_REPORT_TIMES;
extern const uint8 MAX_REQUEST_TIMES;

#endif
