/*
 * ANDES Lab - University of California, Merced
 * This class provides the basic functions of a network node.
 *
 * @author UCM ANDES Lab
 * 
 *
 */
#include <Timer.h>
#include "includes/command.h"
#include "includes/packet.h"
#include "includes/CommandMsg.h"
#include "includes/sendInfo.h"
#include "includes/channels.h"
#include "includes/linkedList.h"
#include "includes/linkState.h"
#include "includes/pair.h"

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
    uses interface Timer<TMilli> as pingTimer;
    uses interface Timer<TMilli> as Timer1; //neighborDiscoveryTimer
    uses interface Timer<TMilli> as lspTimer; //link state timer 
    uses interface Timer<TMilli> as updateNeighbors;
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
    uint16_t neighSeqNum = 0;
    bool netChange = FALSE;
    int maxNodes = 20;
    //int MAX_NODES = 20;
    // Prototypes
    void makePack(pack *Package, uint16_t src, uint16_t dest, uint16_t TTL, uint16_t Protocol, uint16_t seq, uint8_t *payload, uint8_t length);
   
    void printNeighborList();
    void printNeigh(netGRAPH*, int);
    linkedList nodesRecieved;
	linkedList listofNeighbors;
    void neighborDiscovery();
    bool checkPacket(pack Packet);

    //project 2 START 
    
    
	uint16_t linkSequenceNum = 0;
	LSP confirmedList;
	LSP tentativeList;
	netGRAPH lspHashMap[20];
	linkedList lspTracker;
	float cost[20];
	int lastSequenceTracker[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    bool Alive = TRUE;
	void RouteTable();
    int pingTime = 5333;
    
	pingList pingEvents;
    
    //project 1
	void printLinkedList(linkedList *list);
	bool linkedListRemove(linkedList *list, uint32_t iTimer);
	void neighborDiscoveryPacket();
	
	
	//project 2
	void printNetGraph(netGRAPH *list);
	void lspNeighborDiscoveryPacket();
    
	void dijkstra();
	int forwardPacketTo(LSP* list, int dest);
	void printNeighborCosts(netGRAPH *list, uint8_t nodeID);
	
    //------Project 2-------//END

    event void Boot.booted(){
		call AMControl.start();
		linkedListInit(&nodesRecieved);
		dbg(ROUTING_CHANNEL, "Booted\n");
	}
   
    event void pingTimer.fired(){
		checkTimes(&pingEvents, call pingTimer.getNow());
	}

	//checks who are the neighbors
	event void Timer1.fired(){
		if(Alive)neighborDiscoveryPacket();
	}
		
	//checks if the time is still valid to be in the list
	event void updateNeighbors.fired(){
		uint32_t timer = call updateNeighbors.getNow(); 
			
			if(linkedListRemove(&listofNeighbors, timer)){
				printLinkedList(&listofNeighbors);
			}
		//dbg(ROUTING_CHANNEL, "Done checking \n\n");
	}
	
	event void lspTimer.fired(){
		if(Alive)lspNeighborDiscoveryPacket();
	}
    
    
   event void AMControl.startDone(error_t err){
		if(err == SUCCESS){
			call pingTimer.startPeriodic(pingTime + (uint16_t) ((call Random.rand16())%200));
			call Timer1.startPeriodic(pingTime + (uint16_t) ((call Random.rand16())%200));
			call updateNeighbors.startPeriodic(pingTime + (uint16_t)((call Random.rand16())%200));
			call lspTimer.startPeriodic(pingTime + (uint16_t)((call Random.rand16())%200));
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
        if(!Alive){
			dbg(ROUTING_CHANNEL, "Node is turned off\n");
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
                int forwardtoNode;
                
                
                if (myMsg->dest == TOS_NODE_ID) //resend with protocol pingreply for ACK
                {
                    makePack(&sendPackage, myMsg->src, myMsg->dest, MAX_TTL,PROTOCOL_PING,myMsg->seq,myMsg->payload, PACKET_MAX_PAYLOAD_SIZE);
                    // This is when the flooding of a packet has finally led it to it's final destination
                    if(checkPacket(sendPackage)){
                       //dbg(FLOODING_CHANNEL,"Dropping Packet from src: %d to dest: %d with seq num:%d\n", myMsg->src,myMsg->dest,myMsg->seq);
                    }else{
                    
                    dbg(FLOODING_CHANNEL, "Packet has Arrived to destination! %d -> %d seq num: %d\n ", myMsg->src,myMsg->dest, myMsg->seq);
                    dbg(FLOODING_CHANNEL, "Package Payload: %s\n", myMsg->payload);
                    dbg(FLOODING_CHANNEL, "Sending Ping Reply to %d! \n", myMsg->src);
					
					dijkstra();
					
					forwardtoNode = forwardPacketTo(&confirmedList,myMsg->src);
                    
                    dbg(ROUTING_CHANNEL,"Forwarding to %d and src is %d to destination: %d\n", forwardtoNode, TOS_NODE_ID, myMsg->src);
                    makePack(&sendPackage, TOS_NODE_ID, myMsg->src, 20,PROTOCOL_PINGREPLY,myMsg->seq,myMsg->payload, PACKET_MAX_PAYLOAD_SIZE);
                    call Sender.send(sendPackage, forwardtoNode);
                    
                    
                    }
                }
                else //not meant for this node forward to correct nextHop
                {   //int forwardtoNode;
                    makePack(&sendPackage, myMsg->src, myMsg->dest, myMsg->TTL-1, PROTOCOL_PING, myMsg->seq, myMsg->payload, PACKET_MAX_PAYLOAD_SIZE);
                    if(checkPacket(sendPackage)){//return true meaning packet found in SeenPackList
                        dbg(FLOODING_CHANNEL,"ALREADY SEEN: Dropping Packet from src: %d to dest: %d with seq num:%d\n", myMsg->src,myMsg->dest,myMsg->seq);
                        //dbg(FLOODING_CHANNEL,"ALREADY SEEN: Dropping Packet from src: %d to dest: %d\n", myMsg->src,myMsg->dest);
                    }else{ //
                        //makePack(&sendPackage, TOS_NODE_ID, destination, 0, PROTOCOL_PING, seqNum, payload, PACKET_MAX_PAYLOAD_SIZE);
                    //dbg(FLOODING_CHANNEL,"Packet Recieved from %d meant for %d, Sequence Number %d...Rebroadcasting\n",myMsg->src, myMsg->dest, myMsg->seq);
                    //int forwardtoNode;
				       
				        
				        dijkstra();
				        
				        forwardtoNode = forwardPacketTo(&confirmedList,myMsg->dest);
				        
				        if(forwardtoNode == 0) printNeighborCosts(&lspHashMap, TOS_NODE_ID); //must be this node
				        if(forwardtoNode == -1){ //error detected
					        dbg(ROUTING_CHANNEL, "rerunning dikstra \n");
					        dijkstra();
					        forwardtoNode = forwardPacketTo(&confirmedList,myMsg->dest);
					        if(forwardtoNode == -1)
						        dbg(ROUTING_CHANNEL, "Dropping.....\n");
					        else{
						        dbg(ROUTING_CHANNEL,"Forwarding to %d and src is %d \n", forwardtoNode, myMsg->src);
						        call Sender.send(sendPackage, forwardtoNode);
						        
					        }
				        }else{
					        dbg(ROUTING_CHANNEL,"Forwarding to %d and src is %d \n", forwardtoNode, myMsg->src);
					        call Sender.send(sendPackage, forwardtoNode);
					        
				    }
                    //dbg(FLOODING_CHANNEL,"Packet Recieved from %d meant for %d... Rebroadcasting\n",myMsg->src, myMsg->dest);
                    

                    
                    }
                    

                }
            }
            else if (myMsg->dest == AM_BROADCAST_ADDR) //neigbor discovery OR LSP
            {   
                    pair neighbor;
				    uint8_t *tempArray;
				    int i, j;
				    int difference; 
                
                if(myMsg->protocol == PROTOCOL_LINKSTATE){
                    
                   
                    
                    if(!linkedListContains(&lspTracker, myMsg->src, myMsg->seq)){
							if(linkedListSize(&lspTracker) >= 30){
								//dbg(ROUTING_CHANNEL,"Popping front\n");
								pop_front(&lspTracker);	
							}
							temp1.seq = myMsg->seq;
							temp1.src = myMsg->src;
							linkedListPushBack(&lspTracker,temp1);
							netGRAPHInit(&lspHashMap,myMsg->src);
                            
							//dbg(ROUTING_CHANNEL,"LINK STATE PACKET from %d seq#: %d  \n", myMsg->src, myMsg->seq);								
							for(i = 0; i < maxNodes; i++){
								lspHashMap[myMsg->src].cost[i] = myMsg->payload[i];
                                if(myMsg->src == 2){
                                    //dbg(ROUTING_CHANNEL, "Printing out src:%d neighbor:%d  cost:%d \n", myMsg->src, i , myMsg->payload[i]);
                                }
								if(myMsg->payload[i] != -1 && myMsg->payload[i] != 255){
                                    //dbg(ROUTING_CHANNEL, "Printing out src:%d neighbor:%d  cost:%d \n", myMsg->src, i , myMsg->payload[i]);
                                }
									
							}
                            //if(TOS_NODE_ID == 7){
                                //printNetGraph(lspHashMap);
                            //}
							makePack(&sendPackage, myMsg->src, myMsg->dest, myMsg->TTL-1, myMsg->protocol, myMsg->seq, (uint8_t *) myMsg->payload, 20);
							
                            call Sender.send(sendPackage, AM_BROADCAST_ADDR);
							
						}else{ //LSPpacket already seen
                            //dbg(ROUTING_CHANNEL,"LSPPacket already recieved from %d\n", myMsg->src);
                    }
                }else if(myMsg->protocol == PROTOCOL_PINGREPLY){
                        difference = 0;
						
						
						//dbg(NEIGHBOR_CHANNEL, "Received Neighbor reply from %d seq#: %d \n", myMsg->src, myMsg->seq);
						if(!linkedListContains(&listofNeighbors, myMsg->src, myMsg->seq)){
							neighbor.seq = myMsg->seq;
							neighbor.src = myMsg->src;
							neighbor.timer = call Timer1.getNow();
							if(linkedListContainsKey(&listofNeighbors, myMsg->src)){
								linkedListReplace(&listofNeighbors,myMsg->src, myMsg->seq, neighbor.timer); //updates the current time of the node
								//dbg(NEIGHBOR_CHANNEL, "Updating NeighborList................\n\n");
							}
							else{
                                linkedListPushBack(&listofNeighbors,neighbor);
                            }
								
							//dbg(NEIGHBOR_CHANNEL, "NOT IN NEIGHBOR LIST, ADDING\n\n");						
							//project 2 portion
							if(lastSequenceTracker[myMsg->src] < myMsg->seq){
								//calculate the cost of the link in here						
								difference =  myMsg->seq -lastSequenceTracker[myMsg->src];
								lastSequenceTracker[myMsg->src] = myMsg->seq;
								
							}
						}
						else{
							//dbg(NEIGHBOR_CHANNEL, "Already in Neigborlist\n");
						}
                }else{
                        dbg(ROUTING_CHANNEL, "ERROR\n");
                }   
                    
                
            }else if(myMsg->protocol == PROTOCOL_PINGREPLY){ //ack message
                    int forwardtoNode;
                  if(myMsg->dest == TOS_NODE_ID){ //ACK reached source
                      makePack(&sendPackage, myMsg->src, myMsg->dest, myMsg->TTL-1, PROTOCOL_PINGREPLY, myMsg->seq, myMsg->payload, PACKET_MAX_PAYLOAD_SIZE);
                      //dbg(FLOODING_CHANNEL,"Node %d recieved ACK from %d\n", TOS_NODE_ID,myMsg->src);
                      if(!checkPacket(sendPackage)){
                          dbg(FLOODING_CHANNEL,"Node %d recieved ACK from %d\n", TOS_NODE_ID,myMsg->src);
                       //dbg(FLOODING_CHANNEL,"Dropping Packet from src: %d to dest: %d with seq num:%d\n", myMsg->src,myMsg->dest,myMsg->seq);
                        }
                  }else{
                      //dbg(FLOODING_CHANNEL, "Forwarding Ping Reply to %d! \n\n", myMsg->dest);
					
					dijkstra();
					
					forwardtoNode = forwardPacketTo(&confirmedList,myMsg->dest);
                    dbg(ROUTING_CHANNEL,"Forwarding Ping Reply to %d and src is %d to dest: %d\n", forwardtoNode, myMsg->src, myMsg->dest);
                        makePack(&sendPackage, myMsg->src, myMsg->dest, myMsg->TTL - 1,PROTOCOL_PINGREPLY,myMsg->seq,myMsg->payload, PACKET_MAX_PAYLOAD_SIZE);
                        call Sender.send(sendPackage, forwardtoNode);
                  }

            }
            
            return msg;
        }
        dbg(GENERAL_CHANNEL, "Unknown Packet Type %d\n", len);
        return msg;
    }
    
    
    event void CommandHandler.ping(uint16_t destination, uint8_t *payload){
        int forwardtoNode;
        
        dbg(GENERAL_CHANNEL, "PING EVENT \n");
        dbg(ROUTING_CHANNEL,"Ping to %d and src is %d \n", destination, TOS_NODE_ID);
        makePack(&sendPackage, TOS_NODE_ID, destination, 20, PROTOCOL_PING, seqNum, payload, PACKET_MAX_PAYLOAD_SIZE);
        //printNetGraph(lspHashMap);
       
		dijkstra();
					
		forwardtoNode = forwardPacketTo(&confirmedList,destination);
        dbg(ROUTING_CHANNEL,"Forwarding to %d and src is %d \n", forwardtoNode, TOS_NODE_ID);
                    
        call Sender.send(sendPackage, forwardtoNode);
        
        
        seqNum++;
    }
    
    event void CommandHandler.printNeighbors()
    {
        printNeigh(&lspHashMap,TOS_NODE_ID);
    }
    
    event void CommandHandler.printRouteTable(){
        RouteTable();
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
		//dbg(NEIGHBOR_CHANNEL, "Sending seq#: %d\n", neighSeqNum);
		makePack(&sendPackage, TOS_NODE_ID, AM_BROADCAST_ADDR, 20, PROTOCOL_PINGREPLY, neighSeqNum++, (uint8_t *)createMsg,
				sizeof(createMsg));	
		//dbg(ROUTING_CHANNEL, "NeighborDiscovery for %d\n", TOS_NODE_ID);
		call Sender.send(sendPackage,AM_BROADCAST_ADDR);
			
	}	

	//Checks for the node time out
	bool linkedListRemove(linkedList *list, uint32_t iTimer){
		uint8_t i;
		uint8_t j;
		
		bool removed = FALSE;
		for(i = 0; i <list->numValues; i++){
			
			if(list->values[i].timer + 50000 < iTimer ){
				dbg(NEIGHBOR_CHANNEL,"Removing %d from NeighborList\n", list->values[i].src);	
				list->values[i] = list->values[list->numValues-1];
				list->numValues--;
				i--;
				removed =  TRUE;
			}
		}
		return removed;
	}
	
	void printLinkedList(linkedList* list){
		uint8_t i;
		for(i = 0; i<list->numValues; i++){
			dbg(NEIGHBOR_CHANNEL,"NEIGHBOR: %d\n", list->values[i].src);
		}	
	}
	//---- END OF PROJECT 1 IMPLEMENTATIONS

    //---- PROJECT 2 IMPLEMENTATIONS
	void printNetGraph(netGRAPH *list){
		int i,j;
		for(i = 0; i < maxNodes; i++){
			for(j = 0; j < maxNodes; j++){
				if(list[i].cost[j] != 255 && list[i].cost[j] != -1 && list[i].cost[j] != 0)
					dbg(ROUTING_CHANNEL, "src: %d  neighbor: %d cost: %d \n", i, j, list[i].cost[j]);
			}	
		}
		dbg(ROUTING_CHANNEL, "END \n\n");
	}
	
	void printNeighborCosts(netGRAPH *list, uint8_t nodeID) {
		uint8_t i;
		for(i = 0; i < maxNodes; i++) {
			//dbg(ROUTING_CHANNEL, "From %d To %d Costs %d", nodeID, i, list[nodeID].cost[i]);
		}
	}

	void lspNeighborDiscoveryPacket(){
		uint16_t dest;
        
		int i, j;
		uint8_t lspCostList[20] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}; 
        for(j = 0; j < maxNodes; j++){
            //lspCostList[i] = -1;
        }	
		netGRAPHInit(&lspHashMap,TOS_NODE_ID);
        
        
        
		for(i = 0; i < listofNeighbors.numValues; i++){
			
              if(!linkedListIsEmpty(&listofNeighbors)){  

				lspCostList[listofNeighbors.values[i].src] = 10;
				
				//puts the neighbor into the MAP
				lspHashMap[TOS_NODE_ID].cost[listofNeighbors.values[i].src] = 10;
               
                
				//dbg(ROUTING_CHANNEL, "Neighbor: %d and cost %d\n",listofNeighbors.values[i].src, lspCostList[listofNeighbors.values[i].src]);
			}
			else{
                dbg(ROUTING_CHANNEL, "No neighbors yet. \n", listofNeighbors.values[i].src);
            }
				
		}
		memcpy(&dest, "", sizeof(uint8_t));	
		makePack(&sendPackage, TOS_NODE_ID, AM_BROADCAST_ADDR, MAX_TTL, PROTOCOL_LINKSTATE, linkSequenceNum++, (uint8_t *)lspCostList, 20);	
		
        call Sender.send(sendPackage,AM_BROADCAST_ADDR);
		
		//dbg(ROUTING_CHANNEL, "Sending LSP\n");	
		
	}	
		
    void neighborDiscovery(){
        
    
        char* dummyMsg = "NULL\n";

       //dbg(NEIGHBOR_CHANNEL, "Neighbor Discovery: checking node %d list for its neighbors\n", TOS_NODE_ID);
		
			
		
		

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
		LSPinit(&tentativeList); LSPinit(&confirmedList);
		//dbg(ROUTING_CHANNEL,"running dijkstra \n");
        //printNetGraph(lspHashMap);
		LSPPushBack(&tentativeList, temp = (lspTuple){TOS_NODE_ID,0,TOS_NODE_ID});
		//dbg(ROUTING_CHANNEL,"PushBack from tentativeList dest:%d cost:%d nextHop:%d \n", temp.dest, temp.nodeNcost, temp.nextHop);
		while(!LSPIsEmpty(&tentativeList)){
            lspTup = removeMin(&tentativeList);
			if(!LSPContains(&confirmedList,lspTup)) //gets the minCost node from the tentative and removes it, then checks if it's in the confirmed list.
				if(LSPPushBack(&confirmedList,lspTup)){
					//dbg(ROUTING_CHANNEL,"PushBack from confirmedList dest:%d cost:%d nextHop:%d \n", lspTup.dest,lspTup.nodeNcost, lspTup.nextHop);
                }
			for(i = 1; i < 20; i++){
				temp = (lspTuple){i,lspHashMap[lspTup.dest].cost[i]+lspTup.nodeNcost,(lspTup.nextHop == TOS_NODE_ID)?i:lspTup.nextHop};
				if(!LSPContainsDest(&confirmedList, i) && lspHashMap[lspTup.dest].cost[i] != 255 && lspHashMap[i].cost[lspTup.dest] != 255 && lspTupleReplace(&tentativeList,temp,temp.nodeNcost)){
						//dbg(ROUTING_CHANNEL,"Replace from tentativeList dest:%d cost:%d nextHop:%d\n", temp.dest, temp.nodeNcost, temp.nextHop);
                }
				else if(!LSPContainsDest(&confirmedList, i) && lspHashMap[lspTup.dest].cost[i] != 255 && lspHashMap[i].cost[lspTup.dest] != 255 && LSPPushBack(&tentativeList, temp)){
						//dbg(ROUTING_CHANNEL,"PushBack2 from tentativeList dest:%d cost:%d nextHop:%d \n", temp.dest, temp.nodeNcost, temp.nextHop);
                }
			}
		}
		//dbg(ROUTING_CHANNEL, "Printing Routing table\n");
		for(i = 0; i < confirmedList.numValues; i++){
            //dbg(ROUTING_CHANNEL, "dest:%d cost:%d nextHop:%d \n",confirmedList.lspTuples[i].dest,confirmedList.lspTuples[i].nodeNcost,confirmedList.lspTuples[i].nextHop);
        }
			
		//dbg(ROUTING_CHANNEL, "dijkstra done\n");
	}

	int forwardPacketTo(LSP* list, int dest){	
		return LSPLookUp(list,dest);
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
    void RouteTable(){
        int i;	
		lspTuple lspTup, temp;
		LSPinit(&tentativeList); LSPinit(&confirmedList);
		
		LSPPushBack(&tentativeList, temp = (lspTuple){TOS_NODE_ID,0,TOS_NODE_ID});
		
		while(!LSPIsEmpty(&tentativeList)){
            lspTup = removeMin(&tentativeList);
			if(!LSPContains(&confirmedList,lspTup)) //gets the minCost node from the tentative and removes it, then checks if it's in the confirmed list.
				if(LSPPushBack(&confirmedList,lspTup)){
					
                }
			for(i = 1; i < 20; i++){
				temp = (lspTuple){i,lspHashMap[lspTup.dest].cost[i]+lspTup.nodeNcost,(lspTup.nextHop == TOS_NODE_ID)?i:lspTup.nextHop};
				if(!LSPContainsDest(&confirmedList, i) && lspHashMap[lspTup.dest].cost[i] != 255 && lspHashMap[i].cost[lspTup.dest] != 255 && lspTupleReplace(&tentativeList,temp,temp.nodeNcost)){
                        
                }else if(!LSPContainsDest(&confirmedList, i) && lspHashMap[lspTup.dest].cost[i] != 255 && lspHashMap[i].cost[lspTup.dest] != 255 && LSPPushBack(&tentativeList, temp)){
						
                }
			}
		}
		dbg(ROUTING_CHANNEL, "ROUTING TABLE of %d: \n", TOS_NODE_ID);
        bg(ROUTING_CHANNEL, "------------------------------\n");
		for(i = 0; i < confirmedList.numValues; i++)
			dbg(ROUTING_CHANNEL, "dest:%d cost:%d nextHop:%d \n",confirmedList.lspTuples[i].dest,confirmedList.lspTuples[i].nodeNcost,confirmedList.lspTuples[i].nextHop);
		
    }
    void printNeigh(netGRAPH *list, int nodeID) {
		uint8_t i;
        dbg(NEIGHBOR_CHANNEL, "%d Neighbors:\n", TOS_NODE_ID);
		for(i = 0; i < maxNodes; i++) {
            if(list[nodeID].cost[i] > 0 && list[nodeID].cost[i] < 255){
                dbg(NEIGHBOR_CHANNEL, "%d\n", i);
            }
			
		}
	}
    
    

    
	
	
    
}