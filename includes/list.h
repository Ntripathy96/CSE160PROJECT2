//Author: UCM ANDES Lab
#ifndef LIST_H
#define LIST_H

#include "pair.h"
typedef pair dataType;
#define ARRAYSIZE 30
#define MAXNUMVALS ARRAYSIZE

typedef struct linkedList
{	
	dataType values[ARRAYSIZE]; //list of values
	uint8_t numValues;			//number of objects currently in the array
}linkedList;

void linkedListInit(linkedList *cur){
	cur->numValues = 0;	
}

bool linkedListPushBack(linkedList* cur, dataType newVal){
	if(cur->numValues != MAXNUMVALS){
		cur->values[cur->numValues] = newVal;
		++cur->numValues;
		return TRUE;
	}else return FALSE;
}

bool linkedListPushFront(linkedList* cur, dataType newVal){
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

dataType pop_back(linkedList* cur){
	--cur->numValues;
	return cur->values[cur->numValues];
}

dataType pop_front(linkedList* cur){
	dataType returnVal;
	nx_uint8_t i;	
	returnVal = cur->values[0];
	for(i = 1; i < cur->numValues; ++i)
	{
		cur->values[i-1] = cur->values[i];
	}
	--cur->numValues;
	return returnVal;			
}

dataType front(linkedList* cur)
{
	return cur->values[0];
}

dataType back(linkedList * cur)
{
	return cur->values[cur->numValues-1];	
}

bool linkedListIsEmpty(linkedList* cur)
{
	if(cur->numValues == 0)
		return TRUE;
	else
		return FALSE;
}

uint8_t linkedListSize(linkedList* cur){	return cur->numValues;}

void linkedListClear(linkedList* cur){	cur->numValues = 0;}

dataType linkedListGet(linkedList* cur, nx_uint8_t i){	return cur->values[i];}

bool linkedListContains(linkedList* list, uint8_t iSrc, uint8_t iSeq){
	uint8_t i=0;
	for(i; i<list->numValues; i++){
		if(iSeq == list->values[i].seq && iSrc == list->values[i].src) return TRUE;
	}
	return FALSE;
}

bool linkedListContainsKey(linkedList* list, uint8_t iSrc){
	uint8_t i=0;
	for(i; i<list->numValues; i++){
		if(iSrc == list->values[i].src) return TRUE;
	}
	return FALSE;
}

dataType linkedListGetPair(linkedList* list, uint8_t iSrc){
	uint8_t i=0;
	pair derp;
	for(i; i<list->numValues; i++){
		if(iSrc == list->values[i].src) return list->values[i];
	}
	derp.timer = 0;
	return derp;
	
}

void linkedListReplace(linkedList *list, uint8_t iSrc, uint8_t iSeq, uint32_t iTimer){
		uint8_t i;
	for(i = 0; i<list->numValues; i++){
		if(iTimer >= list->values[i].timer && iSrc == list->values[i].src){
			list->values[i].seq = iSeq;
			list->values[i].src = iSrc;
			list->values[i].timer = iTimer;
		}
	}
}

/*
//Checks for the node time out
void linkedListRemove(linkedList *list,uint32_t iTimer){
	uint8_t i;
	uint8_t j;
	for(i = 0; i<=list->numValues; i++){
		//printf("%d > %d \n", iTimer, list->values[i].timer);
		if(iTimer >= list->values[i].timer && list->values[i].timer != 0){
			printf("Removing %d from friendList, last seen at time %d. Time removed: %d \n\n", list->values[i].src, list->values[i].timer, iTimer);
			
			if(list->numValues > 1){
				list->values[i] = list->values[list->numValues-1];		
				list->numValues--;
				i--;
			}
			else
				list->numValues = 0;
		}
	}
	
}
*/
#endif /* LIST_H */