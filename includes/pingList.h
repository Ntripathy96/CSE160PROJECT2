#ifndef PING_LIST_H
#define PING_LIST_H

#include "pingInfo.h"
typedef pingInfo pingType;

#define PINGLIST_SIZE 30
#define PINGLIST_MAXNUMVALS PINGLIST_SIZE

typedef struct pingList{	
	pingType values[PINGLIST_SIZE]; //list of values
	uint8_t numValues;			//number of objects currently in the array
}pingList;

void pingListInit(pingList *cur){
	cur->numValues = 0;	
}

bool pingListPushBack(pingList* cur, pingType newVal){
	if(cur->numValues != PINGLIST_MAXNUMVALS){
		cur->values[cur->numValues] = newVal;
		++cur->numValues;
		return TRUE;	
	}else return FALSE;
}

bool pingListIsEmpty(pingList* cur){
	if(cur->numValues == 0)		return TRUE;
	else	return FALSE;
}

uint8_t pingListSize(pingList* cur){	return cur->numValues;}

void pingListClear(pingList* cur){	cur->numValues = 0;}

pingType pingListGet(pingList* cur, nx_uint8_t i){	return cur->values[i];}

uint16_t pingListGetKey(pingList* list, uint16_t dest, uint8_t *payload){
	uint16_t i;
	for(i=0; i<list->numValues; i++){
		if(payloadEquals(payload, list->values[i].msg)&& list->values[i].dest ==dest){
			return i;
		}
	}
	return 0;
}

bool pingListContains(pingList* list, uint16_t dest, uint8_t *payload){
	uint8_t i=0;
	for(i; i<list->numValues; i++){
		if(payloadEquals(payload, list->values[i].msg) && list->values[i].dest == dest)return TRUE;
	}
	return FALSE;
}

void pingListDelete(pingList* list, uint8_t i){
	for(i; i<list->numValues-1; i++){
		list->values[i]=list->values[i+1];
	}
	list->numValues--;
}

void pingListRemove(pingList* list, uint16_t dest, uint8_t *payload){
	uint8_t i=0;
	for(i=0; i<list->numValues; i++){
		if(payloadEquals(payload, list->values[i].msg)&& list->values[i].dest ==dest){
			pingListDelete(list, i);
			return;
		}
	}
	dbg("pingList", "Unable to remove ping. \n");
}

#endif /* PING_LIST_H */