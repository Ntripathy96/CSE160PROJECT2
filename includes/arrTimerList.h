//Modified version of List.h

#ifndef TIMER_LIST_H
#define TIMER_LIST_H

#include "pair.h"
typedef pair timerDataType;
#define ARRAYSIZE 30
#define MAXNUMVALS ARRAYSIZE
/*
typedef struct arrTimerList
{	
	timerDataType values[ARRAYSIZE]; //list of values
	uint8_t numValues;			//number of objects currently in the array
}arrlist;
void arrTimerListInit(arrlist *cur){
	cur->numValues = 0;	
}
bool arrTimerListPushBack(arrlist* cur, timerDataType newVal){
	if(cur->numValues != MAXNUMVALS){
		cur->values[cur->numValues] = newVal;
		++cur->numValues;
		return TRUE;
	}else return FALSE;
}
bool arrTimerListPushFront(arrlist* cur, timerDataType newVal){
	if(cur->numValues!= MAXNUMVALS){
		uint8_t i;
		for( i = cur->numValues-1; i >= 0; --i){
			cur->values[i+1] = cur->values[i];
		}
		cur->values[0] = newVal;
		++cur->numValues;
		return TRUE;
	}else	return FALSE;
} 
timerDataType pop_backTimer(arrlist* cur){
	--cur->numValues;
	return cur->values[cur->numValues];
}
timerDataType pop_frontTimer(arrlist* cur){
	timerDataType returnVal;
	nx_uint8_t i;	
	returnVal = cur->values[0];
	for(i = 1; i < cur->numValues; ++i)
	{
		cur->values[i-1] = cur->values[i];
	}
	--cur->numValues;
	return returnVal;			
}
timerDataType frontTimer(arrlist* cur)
{
	return cur->values[0];
}
timerDataType backTimer(arrlist * cur)
{
	return cur->values[cur->numValues-1];	
}
bool arrTimerListIsEmpty(arrlist* cur)
{
	if(cur->numValues == 0)
		return TRUE;
	else
		return FALSE;
}
uint8_t arrTimerListSize(arrlist* cur){	return cur->numValues;}
void arrTimerListClear(arrlist* cur){	cur->numValues = 0;}
timerDataType arrTimerListGet(arrlist* cur, nx_uint8_t i){	return cur->values[i];}
bool arrTimerListContains(arrlist* list, uint8_t iSrc, uint8_t iSeq){
	uint8_t i=0;
	for(i; i<list->numValues; i++){
		if(iSeq == list->values[i].seq && iSrc == list->values[i].src) return TRUE;
	}
	return FALSE;
}
void arrTimerListReplace(arrlist *list, uint8_t iSrc, uint8_t iSeq){
		uint8_t i;
	for(i = 0; i<list->numValues; i++){
		if(iSeq == list->values[i].seq && iSrc == list->values[i].src){
			list->values[i].seq = iSeq;
			list->values[i].src = iSrc;
		}
	}
}
//Checks if the 
void arrTimerListRemove(arrlist *list, uint8_t iSrc, uint8_t iTime){
	uint8_t i;
	for(i = 0; i<list->numValues; i++){
		if(iTime >= list->values[i].seq && iSrc == list->values[i].src){
			list->values[i].seq = iTime;
			list->values[i].src = iSrc;
		}
	}
}
*/
#endif /* TIMER_LIST_H */