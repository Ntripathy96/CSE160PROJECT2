/*
 * ANDES Lab - University of California, Merced
 * This class provides the basic functions of a network node.
 *
 * @author UCM ANDES Lab
 * @date   2013/09/03
 *
 */
#include <Timer.h>
#include "includes/command.h"
#include "includes/packet.h"
#include "includes/CommandMsg.h"
#include "includes/sendInfo.h"
#include "includes/channels.h"
#include "includes/list.h"
#include "includes/lspTable.h"
#include "includes/arrTimerList.h"
#include "includes/lspTable.h"
#include "includes/pair.h"
//Ping Includes
#include "includes/pingList.h"
#include "includes/ping.h"


typedef nx_struct neighbor {
    nx_uint16_t Node;
    nx_uint8_t Life;
}neighbor;
    int seqNum = 1;
    //bool printNodeNeighbors = FALSE;

module Node{
    uses interface Boot;
    uses interface Timer<TMilli> as pingTimeoutTimer;
    uses interface Timer<TMilli> as Timer1; //Interface that was wired above.
    uses interface Timer<TMilli> as lspTimer; //link state timer 
    uses interface Timer<TMilli> as neighborUpdateTimer;
    uses interface Random as Random;
    uses interface SplitControl as AMControl;
    uses interface Receive;
    uses interface List<neighbor> as NeighborList;
    uses interface List<pack> as SeenPackList;
    uses interface List<pack> as SeenLspPackList;
    
    //uses interface Hashmap<int> as NeighborList;
    
    
    uses interface SimpleSend as Sender;
    
    uses interface CommandHandler;
}

implementation{
    pack sendPackage;
    //int seqNum = 0;
    bool printNodeNeighbors = FALSE;
    uint16_t neighborSequenceNum = 0;
    bool netChange = FALSE;
    int totalNodes = 11;
    //int MAX_NODES = 20;
    // Prototypes
    void makePack(pack *Package, uint16_t src, uint16_t dest, uint16_t TTL, uint16_t Protocol, uint16_t seq, uint8_t *payload, uint8_t length);
    void printNeighbors();
    void printNeighborList();
    arrlist Received;
	arrlist friendList;
    void neighborDiscovery();
    bool checkPacket(pack Packet);

    //project 2 START 
    
    //---- PROJECT 2 VARIABLES -----
	//We're keeping track of each node with the index. Assume that the index is the name of the node.
	//note: We need to shift the nodes by 1 so that index 0 is keeping track of node 1. (May be reconsidered)
	uint16_t linkSequenceNum = 0;
	lspTable confirmedList;
	lspTable tentativeList;
	lspMap lspMAP[20];
	arrlist lspTracker;
	float cost[20];
	int lastSequenceTracker[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	float totalAverageEMA[20] =  {0.0039215,0.0039215,0.0039215,0.0039215,0.0039215,0.0039215,0.0039215,0.0039215,0.0039215,0.0039215,0.0039215,0.0039215,0.0039215,0.0039215,0.0039215,0.0039215,0.0039215,0.0039215,0.0039215,0.0039215};
    bool isActive = TRUE;
	
	int discoveryPacket = AM_BROADCAST_ADDR;

    //Ping/PingReply Variables
	pingList pings;

    //project 1
	void arrPrintList(arrlist *list);
	bool arrListRemove(arrlist *list, uint32_t iTimer);
	void neighborDiscoveryPacket();
	
	
	//project 2
	void printlspMap(lspMap *list);
	void lspNeighborDiscoveryPacket();
    
	void dijkstra();
	int forwardPacketTo(lspTable* list, int dest);
	void printCostList(lspMap *list, uint8_t nodeID);
	float EMA(float prevEMA, float now,float weight);
    //------Project 2-------//END

    event void Boot.booted(){
		call AMControl.start();
		arrListInit(&Received);
		dbg(ROUTING_CHANNEL, "Booted\n");
	}
   
    event void pingTimeoutTimer.fired(){
		checkTimes(&pings, call pingTimeoutTimer.getNow());
	}

	//checks who are the neighbors
	event void Timer1.fired(){
		if(isActive)neighborDiscoveryPacket();
	}
		
	//checks if the time is still valid to be in the list
	event void neighborUpdateTimer.fired(){
		uint32_t timerCheck = call neighborUpdateTimer.getNow(); //give the node a 50 second margin from the current time.
			//dbg(NEIGHBOR_CHANNEL, "Checking the neighbor %d \n", timerCheck);
			if(arrListRemove(&friendList, timerCheck)){
				//lspNeighborDiscoveryPacket();
				dbg(NEIGHBOR_CHANNEL, "Removed something \n");
				arrPrintList(&friendList);
			}
		dbg(ROUTING_CHANNEL, "Done checking \n\n");
	}
	
	event void lspTimer.fired(){
		if(isActive)lspNeighborDiscoveryPacket();
	}
    
    
   event void AMControl.startDone(error_t err){
		if(err == SUCCESS){
			call pingTimeoutTimer.startPeriodic(PING_TIMER_PERIOD + (uint16_t) ((call Random.rand16())%200));
			call Timer1.startPeriodic(PING_TIMER_PERIOD + (uint16_t) ((call Random.rand16())%200));
			call neighborUpdateTimer.startPeriodic(PING_TIMER_PERIOD + (uint16_t)((call Random.rand16())%200));
			call lspTimer.startPeriodic(PING_TIMER_PERIOD + (uint16_t)((call Random.rand16())%200));
		}else{
			//Retry until successful
			call AMControl.start();
		}
	}
    
    event void AMControl.stopDone(error_t err){
    }
    
    event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len){
        //dbg(GENERAL_CHANNEL, "Packet Received\n");
         
                
                uint16_t size = call NeighborList.size();
        if(!isActive){
			dbg(ROUTING_CHANNEL, "The Node is inactive, packet will not be read.\n");
			return msg;
		}
                

        if(len==sizeof(pack)){
            pack* myMsg=(pack*) payload;
            //dbg(GENERAL_CHANNEL, "Packet received from %d\n",myMsg->src);
            pair temp1;
			pair temp2;
            pair temporary;
            uint8_t srcPort, destPort, destAddr;
            bool derping;
            //dbg(FLOODING_CHANNEL, "Packet being flooded to %d\n",myMsg->dest);
            
            

            if(myMsg->TTL == 0){ //check life of packet
                dbg(FLOODING_CHANNEL,"TTL=0: Dropping Packet\n");
            }
            
            else if (myMsg->protocol == PROTOCOL_PING) //flooding
            {
                // This is what causes the flooding
                
               // dbg(FLOODING_CHANNEL,"Packet Received from %d meant for %d... Rebroadcasting\n",myMsg->src, myMsg->dest);
                int forwardTo;
                
                
                if (myMsg->dest == TOS_NODE_ID) //resend with protocol pingreply for ACK
                {
                    makePack(&sendPackage, myMsg->src, myMsg->dest, MAX_TTL,PROTOCOL_PING,myMsg->seq,myMsg->payload, PACKET_MAX_PAYLOAD_SIZE);
                    // This is when the flooding of a packet has finally led it to it's final destination
                    if(checkPacket(sendPackage)){
                       //dbg(FLOODING_CHANNEL,"Dropping Packet from src: %d to dest: %d with seq num:%d\n", myMsg->src,myMsg->dest,myMsg->seq);
                    }else{
                    
                    dbg(FLOODING_CHANNEL, "Packet has Arrived to destination! %d -> %d seq num: %d\n ", myMsg->src,myMsg->dest, myMsg->seq);
                    dbg(FLOODING_CHANNEL, "Package Payload: %s\n", myMsg->payload);
                    dbg(FLOODING_CHANNEL, "Sending Ping Reply to %d! \n\n", myMsg->src);
					dbg(ROUTING_CHANNEL,"Running dijkstra\n");
					dijkstra();
					dbg(ROUTING_CHANNEL,"END\n\n"); 
					forwardTo = forwardPacketTo(&confirmedList,myMsg->src);
                    
                    dbg(ROUTING_CHANNEL,"Forwarding to %d and src is %d \n", forwardTo, TOS_NODE_ID);
                    makePack(&sendPackage, TOS_NODE_ID, myMsg->src, 20,PROTOCOL_PINGREPLY,myMsg->seq,myMsg->payload, PACKET_MAX_PAYLOAD_SIZE);
                    call Sender.send(sendPackage, forwardTo);
                    
                    
                    }
                }
                else //not meant for this node forward to correct nextHop
                {   //int forwardTo;
                    makePack(&sendPackage, myMsg->src, myMsg->dest, myMsg->TTL-1, PROTOCOL_PING, myMsg->seq, myMsg->payload, PACKET_MAX_PAYLOAD_SIZE);
                    if(checkPacket(sendPackage)){//return true meaning packet found in SeenPackList
                        dbg(FLOODING_CHANNEL,"ALREADY SEEN: Dropping Packet from src: %d to dest: %d with seq num:%d\n", myMsg->src,myMsg->dest,myMsg->seq);
                        //dbg(FLOODING_CHANNEL,"ALREADY SEEN: Dropping Packet from src: %d to dest: %d\n", myMsg->src,myMsg->dest);
                    }else{ //
                        //makePack(&sendPackage, TOS_NODE_ID, destination, 0, PROTOCOL_PING, seqNum, payload, PACKET_MAX_PAYLOAD_SIZE);
                    dbg(FLOODING_CHANNEL,"Packet Recieved from %d meant for %d, Sequence Number %d...Rebroadcasting\n",myMsg->src, myMsg->dest, myMsg->seq);
                    //int forwardTo;
				       
				        dbg(ROUTING_CHANNEL,"Running dijkstra\n");
				            dijkstra();
				        dbg(ROUTING_CHANNEL,"END\n\n"); 
				        forwardTo = forwardPacketTo(&confirmedList,myMsg->dest);
				        dbg(ROUTING_CHANNEL,"Forwarding to %d and src is %d \n", forwardTo, myMsg->src);
				        if(forwardTo == 0) printCostList(&lspMAP, TOS_NODE_ID);
				        if(forwardTo == -1){
					        dbg(ROUTING_CHANNEL, "rechecking \n");
					        dijkstra();
					        forwardTo = forwardPacketTo(&confirmedList,myMsg->dest);
					        if(forwardTo == -1)
						        dbg(ROUTING_CHANNEL, "Dropping for reals\n");
					        else{
						        dbg(ROUTING_CHANNEL,"Forwarding to %d and src is %d \n", forwardTo, TOS_NODE_ID);
						        call Sender.send(sendPackage, forwardTo);
						        
					        }
				        }
				    else{
					        dbg(ROUTING_CHANNEL,"Forwarding to %d and src is %d \n", forwardTo, TOS_NODE_ID);
					        call Sender.send(sendPackage, forwardTo);
					        
				    }
                    //dbg(FLOODING_CHANNEL,"Packet Recieved from %d meant for %d... Rebroadcasting\n",myMsg->src, myMsg->dest);
                    

                    
                    }
                    

                }
            }
            else if (myMsg->dest == AM_BROADCAST_ADDR) //neigbor discovery OR LSP
            {   
                    pair friendListInfo;
				    uint8_t *tempArray;
				    int i, j;
				    int difference; 
                
                if(myMsg->protocol == PROTOCOL_LINKSTATE){
                    
                   
                    
                    if(!arrListContains(&lspTracker, myMsg->src, myMsg->seq)){
							if(arrListSize(&lspTracker) >= 30){
								dbg(ROUTING_CHANNEL,"Popping front\n");
								pop_front(&lspTracker);	
							}
							temp1.seq = myMsg->seq;
							temp1.src = myMsg->src;
							arrListPushBack(&lspTracker,temp1);
							lspMapInit(&lspMAP,myMsg->src);
							dbg(ROUTING_CHANNEL,"LINK STATE OF GREATNESS. FLOODING THE NETWORK from %d seq#: %d :< \n", myMsg->src, myMsg->seq);								
							for(i = 0; i < totalNodes; i++){
								lspMAP[myMsg->src].cost[i] = myMsg->payload[i];
								if(myMsg->payload[i] != -1 && myMsg->payload[i] != 0)
									dbg(ROUTING_CHANNEL, "Printing out src:%d neighbor:%d  cost:%d \n", myMsg->src, i , myMsg->payload[i]);
							}
							makePack(&sendPackage, myMsg->src, myMsg->dest, myMsg->TTL-1, myMsg->protocol, myMsg->seq, (uint8_t *) myMsg->payload, 20);
							
                            call Sender.send(sendPackage, AM_BROADCAST_ADDR);
							
						}else{ //LSPpacket already seen
                            dbg(ROUTING_CHANNEL,"LSPPacket already recieved from %d\n", myMsg->src);
                    }
                }else if(myMsg->protocol == PROTOCOL_PINGREPLY){
                        difference = 0;
						//The packet drops usually happen in the pingreply section
						dbg(NEIGHBOR_CHANNEL, "PingReply Received: That's mean :< %d. \n", myMsg->src);
						dbg(NEIGHBOR_CHANNEL, "Received Ping reply seq#: %d \n", myMsg->seq);
						if(!arrListContains(&friendList, myMsg->src, myMsg->seq)){
							friendListInfo.seq = myMsg->seq;
							friendListInfo.src = myMsg->src;
							friendListInfo.timer = call Timer1.getNow();
							if(arrListContainsKey(&friendList, myMsg->src)){
								arrListReplace(&friendList,myMsg->src, myMsg->seq, friendListInfo.timer); //updates the current time of the node
								dbg(NEIGHBOR_CHANNEL, "---------------Updating my friendList---------------\n\n");
							}
							else
								arrListPushBack(&friendList,friendListInfo);
							dbg(NEIGHBOR_CHANNEL, "NOT IN THE LIST, ADDING: Adding to my FriendList anyways T_T \n\n");						
							//project 2 portion
							if(lastSequenceTracker[myMsg->src] < myMsg->seq){
								//calculate the cost of the link in here						
								difference =  myMsg->seq -lastSequenceTracker[myMsg->src];
								lastSequenceTracker[myMsg->src] = myMsg->seq;
								if(myMsg->seq <= 1)
									totalAverageEMA[myMsg->src] = EMA(1.0,1.0,1.0);		
								else							
									totalAverageEMA[myMsg->src] = EMA(totalAverageEMA[myMsg->src],1,1/difference);			
							}
						}
						else{
							dbg(NEIGHBOR_CHANNEL, "Oh you're already in my FriendList? :D");
						}
                }else{
                        dbg(ROUTING_CHANNEL, "ERROR\n");
                }   
                    
                
            }else if(myMsg->protocol == PROTOCOL_PINGREPLY){ //ack message
                    int forwardTo;
                  if(myMsg->dest == TOS_NODE_ID){ //ACK reached source
                      makePack(&sendPackage, myMsg->src, myMsg->dest, myMsg->TTL-1, PROTOCOL_PINGREPLY, myMsg->seq, myMsg->payload, PACKET_MAX_PAYLOAD_SIZE);
                      //dbg(FLOODING_CHANNEL,"Node %d recieved ACK from %d\n", TOS_NODE_ID,myMsg->src);
                      if(!checkPacket(sendPackage)){
                          dbg(FLOODING_CHANNEL,"Node %d recieved ACK from %d\n", TOS_NODE_ID,myMsg->src);
                       //dbg(FLOODING_CHANNEL,"Dropping Packet from src: %d to dest: %d with seq num:%d\n", myMsg->src,myMsg->dest,myMsg->seq);
                        }
                  }else{
                      dbg(FLOODING_CHANNEL, "Sending Ping Reply to %d! \n\n", myMsg->src);
					dbg(ROUTING_CHANNEL,"Running dijkstra\n");
					dijkstra();
					dbg(ROUTING_CHANNEL,"END\n\n"); 
					forwardTo = forwardPacketTo(&confirmedList,myMsg->src);
                    dbg(ROUTING_CHANNEL,"Forwarding to %d and src is %d \n", forwardTo, TOS_NODE_ID);
                        makePack(&sendPackage, myMsg->src, myMsg->dest, myMsg->TTL - 1,PROTOCOL_PINGREPLY,myMsg->seq,myMsg->payload, PACKET_MAX_PAYLOAD_SIZE);
                        call Sender.send(sendPackage, forwardTo);
                  }

            }
            
            return msg;
        }
        dbg(GENERAL_CHANNEL, "Unknown Packet Type %d\n", len);
        return msg;
    }
    
    
    event void CommandHandler.ping(uint16_t destination, uint8_t *payload){
        int forwardTo;
        dbg(GENERAL_CHANNEL, "PING EVENT \n");
        
        makePack(&sendPackage, TOS_NODE_ID, destination, 20, PROTOCOL_PING, seqNum, payload, PACKET_MAX_PAYLOAD_SIZE);
        dbg(ROUTING_CHANNEL,"Running dijkstra\n");
					dijkstra();
					dbg(ROUTING_CHANNEL,"END\n\n\n\n\n"); 
					forwardTo = forwardPacketTo(&confirmedList,destination);
                    dbg(ROUTING_CHANNEL,"Forwarding to %d and src is %d \n", forwardTo, TOS_NODE_ID);
        call Sender.send(sendPackage, forwardTo);
        
        
        seqNum++;
    }
    
    event void CommandHandler.printNeighbors()
    {
        printNeighborList();
    }
    
    event void CommandHandler.printRouteTable(){
        dijkstra();
    }
    
    event void CommandHandler.printLinkState(){}
    
    event void CommandHandler.printDistanceVector(){}
    
    event void CommandHandler.setTestServer(){}
    
    event void CommandHandler.setTestClient(){}
    
    event void CommandHandler.setAppServer(){}
    
    event void CommandHandler.setAppClient(){}
    
    void makePack(pack *Package, uint16_t src, uint16_t dest, uint16_t TTL, uint16_t protocol, uint16_t seq, uint8_t* payload, uint8_t length){
        Package->src = src;
        Package->dest = dest;
        Package->TTL = TTL;
        Package->seq = seq;
        Package->protocol = protocol;
        memcpy(Package->payload, payload, length);
    }

    bool checkPacket(pack Packet){
            pack PacketMatch;
            //pack* package_PTR = &Packet;
            //pack Packet = Packet;
            if(call SeenPackList.isEmpty()){
            
                call SeenPackList.pushfront(Packet);
                return FALSE;
            }else{
                int i;
                int size = call SeenPackList.size();
                for(i = 0; i < size; i++){
                    PacketMatch = call SeenPackList.get(i);
                    if( (PacketMatch.src == Packet.src) && (PacketMatch.dest == Packet.dest) && (PacketMatch.seq == Packet.seq) && (PacketMatch.protocol== Packet.protocol)){
                        //dbg(FLOODING_CHANNEL,"Packet src %d vs PacketMatch src %d\n", Packet->src,PacketMatch->src);
                        //dbg(FLOODING_CHANNEL,"Packet destination %d vs PacketMatch dest %d\n", Packet->dest,PacketMatch->dest);
                        //dbg(FLOODING_CHANNEL,"Packet seq %d vs PacketMatch seq %d\n", Packet->seq,PacketMatch->seq);
                        //call SeenPackList.remove(i);
                        return TRUE; //packet is found in list and has already been seen by node.

                    }

                }
    
                
            }
            //other wise packet not found and we need to push it into seen pack list
                call SeenPackList.pushfront(Packet);
                return FALSE;
    }
    
    //---- Project 1 Implementations

	void neighborDiscoveryPacket(){
		pack discoveryPackage;
		uint8_t createMsg[PACKET_MAX_PAYLOAD_SIZE];
		uint16_t dest;
		memcpy(&createMsg, "", sizeof(PACKET_MAX_PAYLOAD_SIZE));
		memcpy(&dest, "", sizeof(uint8_t));
		dbg(NEIGHBOR_CHANNEL, "Sending seq#: %d\n", neighborSequenceNum);
		makePack(&sendPackage, TOS_NODE_ID, discoveryPacket, MAX_TTL, PROTOCOL_PING, neighborSequenceNum++, (uint8_t *)createMsg,
				sizeof(createMsg));	
		dbg(ROUTING_CHANNEL, "Hi, is anyone there? :D \n");
		call Sender.send(sendPackage,AM_BROADCAST_ADDR);
			
	}	

	//Checks for the node time out
	bool arrListRemove(arrlist *list, uint32_t iTimer){
		uint8_t i;
		uint8_t j;
		double timeOut;
		bool success = FALSE;
		for(i = 0; i <list->numValues; i++){
			timeOut = iTimer - list->values[i].timer;
			if(list->values[i].timer + 50000 < iTimer ){
				dbg(NEIGHBOR_CHANNEL,"Removing %d from friendList, last seen at time %d. Time removed: %d \n", list->values[i].src, list->values[i].timer, iTimer);	
				list->values[i] = list->values[list->numValues-1];
				list->numValues--;
				i--;
				success =  TRUE;
			}
		}
		return success;
	}
	
	void arrPrintList(arrlist* list){
		uint8_t i;
		for(i = 0; i<list->numValues; i++){
			dbg(NEIGHBOR_CHANNEL,"I think I am friends with %d and the last time we met was %d \n", list->values[i].src, list->values[i].timer);
		}	
	}
	//---- END OF PROJECT 1 IMPLEMENTATIONS

    //---- PROJECT 2 IMPLEMENTATIONS
	void printlspMap(lspMap *list){
		int i,j;
		for(i = 0; i < totalNodes; i++){
			for(j = 0; j < totalNodes; j++){
				if(list[i].cost[j] != 0 && list[i].cost[j] != -1)
					dbg(ROUTING_CHANNEL, "src: %d  neighbor: %d cost: %d \n", i, j, list[i].cost[j]);
			}	
		}
		dbg(ROUTING_CHANNEL, "END \n\n");
	}
	
	void printCostList(lspMap *list, uint8_t nodeID) {
		uint8_t i;
		for(i = 0; i < totalNodes; i++) {
			dbg(ROUTING_CHANNEL, "From %d To %d Costs %d", nodeID, i, list[nodeID].cost[i]);
		}
	}

	void lspNeighborDiscoveryPacket(){
		uint16_t dest;
		int i;
		uint8_t lspCostList[20] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};	
		lspMapInit(&lspMAP,TOS_NODE_ID);
		for(i = 0; i < friendList.numValues; i++){
			if(1/totalAverageEMA[friendList.values[i].src]*10 < 255){
				lspCostList[friendList.values[i].src] = 1/totalAverageEMA[friendList.values[i].src]*10;
				dbg(ROUTING_CHANNEL, "Cost to %d is %d %f %f\n", friendList.values[i].src, lspCostList[friendList.values[i].src], 1/totalAverageEMA[friendList.values[i].src]*10,totalAverageEMA[friendList.values[i].src]);
				//puts the neighbor into the MAP
				lspMAP[TOS_NODE_ID].cost[friendList.values[i].src] = 1/totalAverageEMA[friendList.values[i].src]*10;
				dbg(ROUTING_CHANNEL, "Priting neighbors: %d %d\n",friendList.values[i].src, lspCostList[friendList.values[i].src]);
			}
			else
				dbg(ROUTING_CHANNEL, "Cost is too big, %d is not my neighbor yet. \n", friendList.values[i].src);
		}
		memcpy(&dest, "", sizeof(uint8_t));	
		makePack(&sendPackage, TOS_NODE_ID, discoveryPacket, MAX_TTL, PROTOCOL_LINKSTATE, linkSequenceNum++, (uint8_t *)lspCostList, 20);	
		
        call Sender.send(sendPackage,AM_BROADCAST_ADDR);
		
		dbg(ROUTING_CHANNEL, "Sending LSPs EVERYWHERE \n");	
		dbg(ROUTING_CHANNEL, "END \n\n");
	}	
		
    void neighborDiscovery(){
        
    
        char* dummyMsg = "NULL\n";

       dbg(NEIGHBOR_CHANNEL, "Neighbor Discovery: checking node %d list for its neighbors\n", TOS_NODE_ID);
		
			
		
		

        makePack(&sendPackage, TOS_NODE_ID, AM_BROADCAST_ADDR, MAX_TTL, PROTOCOL_PINGREPLY, -1, dummyMsg, PACKET_MAX_PAYLOAD_SIZE);
        call Sender.send(sendPackage, AM_BROADCAST_ADDR);
        
        if (printNodeNeighbors)
        {
            printNodeNeighbors = FALSE;
            printNeighborList();
        }
        else
        {
            printNodeNeighbors = TRUE;
            
        }
    }

    void printNeighborList()
    {
        int i;
        neighbor neighPtr;
        if(call NeighborList.size() == 0 ){
            dbg(NEIGHBOR_CHANNEL,"No neighbors for node %d\n", TOS_NODE_ID);

        }else{
            dbg(NEIGHBOR_CHANNEL,"Neighbors for node %d\n",TOS_NODE_ID);
             
        for(i = 0; i < call NeighborList.size(); i++)
        {
            neighPtr = call NeighborList.get(i);
            dbg(NEIGHBOR_CHANNEL,"NeighborNode: %d\n", neighPtr.Node);
        }
        }
        
    }
    void dijkstra(){
		int i;	
		lspTuple lspTup, temp;
		lspTableinit(&tentativeList); lspTableinit(&confirmedList);
		dbg(ROUTING_CHANNEL,"start of dijkstra \n");
		lspTablePushBack(&tentativeList, temp = (lspTuple){TOS_NODE_ID,0,TOS_NODE_ID});
		dbg(ROUTING_CHANNEL,"PushBack from tentativeList dest:%d cost:%d nextHop:%d \n", temp.dest, temp.nodeNcost, temp.nextHop);
		while(!lspTableIsEmpty(&tentativeList)){
			if(!lspTableContains(&confirmedList,lspTup = lspTupleRemoveMinCost(&tentativeList))) //gets the minCost node from the tentative and removes it, then checks if it's in the confirmed list.
				if(lspTablePushBack(&confirmedList,lspTup))
					dbg(ROUTING_CHANNEL,"PushBack from confirmedList dest:%d cost:%d nextHop:%d \n", lspTup.dest,lspTup.nodeNcost, lspTup.nextHop);
			for(i = 1; i < 20; i++){
				temp = (lspTuple){i,lspMAP[lspTup.dest].cost[i]+lspTup.nodeNcost,(lspTup.nextHop == TOS_NODE_ID)?i:lspTup.nextHop};
				if(!lspTableContainsDest(&confirmedList, i) && lspMAP[lspTup.dest].cost[i] != 255 && lspMAP[i].cost[lspTup.dest] != 255 && lspTupleReplace(&tentativeList,temp,temp.nodeNcost))
						dbg(ROUTING_CHANNEL,"Replace from tentativeList dest:%d cost:%d nextHop:%d\n", temp.dest, temp.nodeNcost, temp.nextHop);
				else if(!lspTableContainsDest(&confirmedList, i) && lspMAP[lspTup.dest].cost[i] != 255 && lspMAP[i].cost[lspTup.dest] != 255 && lspTablePushBack(&tentativeList, temp))
						dbg(ROUTING_CHANNEL,"PushBack from tentativeList dest:%d cost:%d nextHop:%d \n", temp.dest, temp.nodeNcost, temp.nextHop);
			}
		}
		dbg(ROUTING_CHANNEL, "Printing the ROUTING_CHANNEL table! \n");
		for(i = 0; i < confirmedList.numValues; i++)
			dbg(ROUTING_CHANNEL, "dest:%d cost:%d nextHop:%d \n",confirmedList.lspTuples[i].dest,confirmedList.lspTuples[i].nodeNcost,confirmedList.lspTuples[i].nextHop);
		dbg(ROUTING_CHANNEL, "End of dijkstra! \n");
	}

	int forwardPacketTo(lspTable* list, int dest){	
		return lspTableLookUp(list,dest);
	}
	
	
	/**
	 * let S_1 = Y_1
	 * Exponential Moving Average
	 * S_t = alpha*Y_t + (1-alpha)*S_(t-1)
	 */	
	float EMA(float prevEMA, float now,float weight){
		float alpha = 0.5*weight;
		float averageEMA = alpha*now + (1-alpha)*prevEMA;
		return averageEMA;
	}


    
    
    
    
    
    bool checkSeenLspPacks(pack Packet){
        pack PacketMatch;
            //pack* package_PTR = &Packet;
            //pack Packet = Packet;
            if(call SeenLspPackList.isEmpty()){
            
                call SeenLspPackList.pushfront(Packet);
                return FALSE;
            }else{
                int i;
                int size = call SeenLspPackList.size();
                for(i = 0; i < size; i++){
                    PacketMatch = call SeenLspPackList.get(i);//check for lsp from a certain node
                    if( (PacketMatch.src == Packet.src) && (PacketMatch.protocol == Packet.protocol)){
                        //dbg(ROUTING_CHANNEL,"LspPacket src %d vs LspPacketMatch src %d\n", Packet.src,PacketMatch.src);
                        //dbg(ROUTING_CHANNEL,"Packet destination %d vs PacketMatch dest %d\n", Packet->dest,PacketMatch->dest);
                        //dbg(ROUTING_CHANNEL,"LSPPacket seq %d vs LSPPacketMatch seq %d\n", Packet.seq,PacketMatch.seq);
                        //call SeenPackList.remove(i);
                        //check if current lsp seqnum is greater or less 
                        if(PacketMatch.seq == Packet.seq) return TRUE;//already in list
                        if(PacketMatch.seq < Packet.seq){//we got a new and updated lsp add to list     
                            call SeenLspPackList.remove(i);
                            call SeenLspPackList.pushback(Packet);
                            return FALSE;
                        }
                        return TRUE; //packet is found in list and has already been seen by node.

                    }

                }
    
                
            }
            //other wise packet not found and we need to push it into seen pack list
                call SeenLspPackList.pushfront(Packet);
                return FALSE;
    }
    
    

    
	
	
    
}