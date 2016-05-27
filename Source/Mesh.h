#ifndef _MESH_H_
#define _MESH_H_

#include "Config.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Macro Definition
//
////////////////////////////////////////////////////////////////////////////////////////////////////
// packet format:		Length  To_ID  From_ID  Type  Reserved  HopNum  Route[Master Node1 ... Noden]  Data[Data0 ... Datam]
#if MAX_NODE_NUMBER < 256
	typedef unsigned char address_size;
	#define ADDRESS_SIZE_IN_BYTE		1
#else
	typedef unsigned short int address_size;
	#define ADDRESS_SIZE_IN_BYTE		2
#endif

#define TO_ID_IN_BYTE					ADDRESS_SIZE_IN_BYTE
#define FROM_ID_IN_BYTE					ADDRESS_SIZE_IN_BYTE

#define BROADCAST_ADDRESS				0x00	// broadcast address for discovery
#define MASTER_ADDRESS					0x01	// master address

#define MAX_NETWORK_LEVEL				5

#define MAX_HOP_NUMBER					5		// max number of hops

#define MAX_DATA_LENGTH					48		// max length of data part, including route field and data field
												// data field contains different kinds of value in different type of packet

#define HEADER_LENGTH					(1 + TO_ID_IN_BYTE + FROM_ID_IN_BYTE + 1 + 1 + 1)		// length + To_ID + From_ID + type + reserved + hopnum

#define MAX_ROUTE_LENGTH				((MAX_HOP_NUMBER + 1) * ADDRESS_SIZE_IN_BYTE)			// master + route list

#define MAX_PAYLOAD_LENGTH				(MAX_ROUTE_LENGTH + MAX_DATA_LENGTH)					// route list + data field

#define MAX_PACKET_LENGTH				(HEADER_LENGTH + MAX_PAYLOAD_LENGTH)					// header + payload

#define LINK_TABLE_LENGTH				(MAX_NODE_NUMBER * (MAX_NODE_NUMBER - 1)) / 2

#define NO_CONNECTION					0x80
#define NONE							0xFF

#define	IN_NET							0x80
#define NOT_IN_NET						0x00

#define WIRELESS_FLAG					0x80
#define	RE_INIT_FLAG					0x40

#define DISCOVERY_FLAG					0x80
#define NEW_NODE_FLAG					0x40
#define DATA_REQUEST_FLAG				0x20
#define MAINTENANCE_FLAG				0x10
#define TOPOLOGY_FLAG					0x08

#define JOIN_FLAG						0x80
#define DATA_FLAG						0x40
#define DELAY_JOIN_REPORT_FLAG			0x20
#define RESPONSE_FLAG					0x10

#define DOWN_LINK						0x01
#define UP_LINK							0x02

////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Enumeration Definition
//
////////////////////////////////////////////////////////////////////////////////////////////////////
// state of master
enum
{
	MASTER_STATE_INITIAL						= 0,
	MASTER_STATE_WAIT							= 1,
	MASTER_STATE_WAIT_FOR_DISCOVERY_RESPONSE	= 2,
	MASTER_STATE_WAIT_FOR_DATA_RESPONSE			= 3,
	MASTER_STATE_WAIT_FOR_CALL_RESPONSE			= 4
};

// state of slave in downlink
enum
{
	SLAVE_DOWNLINK_STATE_WAIT							= 0,
	SLAVE_DOWNLINK_STATE_WAIT_FOR_DISCOVERY_RESPONSE	= 1
};

// state of slave in uplink
enum
{
	SLAVE_UPLINK_STATE_INITIAL							= 0,
	SLAVE_UPLINK_STATE_WAIT_FOR_JOIN_RESPONSE			= 1,
	SLAVE_UPLINK_STATE_WAIT								= 2,
	SLAVE_UPLINK_STATE_WAIT_FOR_DATA_CONFIRMATION		= 3,
	SLAVE_UPLINK_STATE_WAIT_FOR_JOIN_CONFIRMATION		= 4
};

// command set
enum
{
    COMMAND_DISCOVERY_REQUEST		= 0x00,
    COMMAND_DISCOVERY_BROADCAST		= 0x01,
    COMMAND_DISCOVERY_RESPONSE		= 0x02,	
    COMMAND_DISCOVERY_NONRESPONSE	= 0x03,
    COMMAND_DATA_REQUEST			= 0x04,
	COMMAND_DATA_RESPONSE			= 0x05,
	COMMAND_LOCK_REQUEST			= 0x06,
	COMMAND_LOCK_RESPONSE			= 0x07,
	COMMAND_UNLOCK_REQUEST			= 0x08,
	COMMAND_UNLOCK_RESPONSE			= 0x09,
	COMMAND_REMEMBER_REQUEST		= 0x0A,
	COMMAND_REMEMBER_RESPONSE		= 0x0B,
	COMMAND_FORGET_REQUEST			= 0x0C,
	COMMAND_FORGET_RESPONSE			= 0x0D,
	COMMAND_CALL_REQUEST			= 0x0E,
	COMMAND_CALL_RESPONSE			= 0x0F,

    COMMAND_JOIN_REQUEST			= 0x80,
    COMMAND_JOIN_RESPONSE			= 0x81,
    COMMAND_JOIN_REPORT				= 0x82,
    COMMAND_JOIN_CONFIRMATION		= 0x83,
    COMMAND_DATA_REPORT				= 0x84,
	COMMAND_DATA_CONFIRMATION		= 0x85
};

////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Structure Definition
//
////////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct
{
    uint8			Length;				// length of the packet, excluding "Length" byte itself
    address_size	To_ID;				// next hop address
    address_size	From_ID;			// last hop address
    uint8			Type;				// command
    uint8			Reserved;			// reserved
    uint8			HopNum;				// hop number
}Mesh_Head;

typedef struct
{
	Mesh_Head	Header;
	uint8		Payload[MAX_PAYLOAD_LENGTH];
}Mesh_Packet;

typedef union
{
	Mesh_Packet	Packet;
	uint8		Array[MAX_PACKET_LENGTH];
}Mesh_Packet_Union;

typedef struct
{
	uint16			SystemTime;
	uint8			ms_10;
	uint8			Status;
//	uint8			Wireless_RSSI;
	uint16			Last_TRX_Time;
	uint8			Common_Flag;
#ifdef MASTER
	uint8			Master_State;
	uint16			Master_Counter;
	address_size	Data_Request_ID;
	address_size	Last_Request_ID;
	uint8			Last_Request_Type;
	address_size	Maintain_List[MAX_ROUTE_LENGTH];
	uint8			Maintain_Index;
	uint8			Master_Flag;
	uint8			Discovery_Enable;
	uint8			Request_Times;
#endif

#ifdef SLAVE
	address_size	Father_ID;
	uint8			Uplink_State;
	uint16			Uplink_Counter;
	uint8			Downlink_State;
	uint16			Downlink_Counter;
	address_size	Report_NewNode;
//	uint8			Report_RSSI;
	address_size	Delay_NewNode;
//	uint8			Delay_RSSI;
	address_size	Buffered_Route[MAX_ROUTE_LENGTH];
	uint8			Buffered_HopNum;
//	uint8			Buffered_RSSI;
	uint8			Data_Report_Times;
	uint8			Join_Report_Times;
	uint8			Response_Bit[MAX_NODE_NUMBER_DIV8];
	uint8			Child_Bit[MAX_NODE_NUMBER_DIV8];
	uint8			Slave_Flag;
#endif
}Node_Attribution;

////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Variable Declaration
//
////////////////////////////////////////////////////////////////////////////////////////////////////
extern Mesh_Packet_Union	rx_frame;
extern Mesh_Packet_Union 	tx_frame;

extern Node_Attribution 	attr;

#ifdef MASTER
extern uint8				link_quality[];
extern address_size			s_prev_node[];
extern address_size			s_node_dist[];
extern uint8				s_registed_node[];
extern uint8				queue_count;
extern uint8				s_probed_node[];
extern address_size			discovery_current_probing_node;
#endif

#ifdef SNIFFER
extern uint8 print_buf[];
extern uint8 print_head;
extern uint8 print_tail;
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Function Declaration
//
////////////////////////////////////////////////////////////////////////////////////////////////////
void Mesh_Init(void);
void Mesh_Run(void);
void Mesh_Decode(void);
uint8 Mesh_Find_Node(const address_size *route, uint8 hopnum, address_size address, uint8 *index);
void Mesh_Reverse_Route(const address_size *route, uint8 hopnum, address_size *reversed_route);
uint8 Mesh_Random(uint8 min, uint8 max);
void Mesh_Enqueue(address_size node, uint8* parray, uint8* pqueue_len);
uint8 Mesh_Checkqueue(address_size node, const uint8* parray);
void Mesh_Package(uint8 length, address_size to_id, address_size from_id, uint8 type, uint8 reserved, uint8 hopnum, const address_size *route, uint8 datalength, const uint8 *data);
#ifdef MASTER
void Mesh_Set_LQI(address_size start, address_size end, uint8 lqi, uint8* link_table);
uint8 Mesh_Get_LQI(address_size start, address_size end, const uint8* link_table);
uint8 Mesh_Dequeue(uint8* parray, uint8* pqueue_len, address_size *pnode);
int16 Mesh_Get_Realcost(uint8 lqi);
int8 Mesh_Find_Best_Path(address_size start, address_size end, address_size* path, const uint8* link_table);
void Mesh_Maintenance(address_size target_node);
void Mesh_Discovery(void);
#endif

#endif
