#ifndef PING_H
#define PING_H
#include <Timer.h>
#include "pingList.h"
#include "pingInfo.h"

enum{
	PING_TIMER_PERIOD=5333,
	PING_TIMEOUT = 5000 //Time out in 5 seconds
};

void checkTimes(pingList *pings, uint32_t currentTime){
	uint8_t i=0;
	pingInfo temp;
	for(i; i<pingListSize(pings); i++){
		temp=pingListGet(pings,i);
		if(temp.timeSent+PING_TIMEOUT< currentTime){
			dbg("genDebug", "Ping Lost!\n Msg: %s", temp.msg);
			
			pingListDelete(pings, i);
		}
	}
}


#endif /* PING_H */