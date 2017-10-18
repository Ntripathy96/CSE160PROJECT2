#ifndef PING_INFO_H
#define PING_INFO_H
#include "packet.h"

typedef struct pingInfo{
	uint16_t dest;
	uint32_t timeSent;
	uint8_t msg[PACKET_MAX_PAYLOAD_SIZE];
}pingInfo;

bool payloadEquals(uint8_t *A, uint8_t *B){
	uint16_t i;
	for(i=0; i<PACKET_MAX_PAYLOAD_SIZE; i++){
		if(A[i]!=B[i]){
			dbg("genDebug", "PAYLOADS DO NOT EQUAL AT POINT %lu, %hhu==%hhu", i, A[i], B[i]);
			return FALSE;
		}
	}
	return TRUE;
}

pingInfo createPingInfo(uint16_t dest, uint32_t timeSent, uint8_t *payload){
	pingInfo info;
	info.dest=dest;
	info.timeSent=timeSent;
	memcpy(&(info.msg), (payload), PACKET_MAX_PAYLOAD_SIZE);
	
	return info;
}
#endif /* PING_INFO_H */