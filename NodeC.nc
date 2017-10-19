/**
 * ANDES Lab - University of California, Merced
 * This class provides the basic functions of a network node.
 *
 * @author UCM ANDES Lab
 * @date   2013/09/03
 *
 */

#include <Timer.h>
#include "includes/CommandMsg.h"
#include "includes/packet.h"

configuration NodeC{
}
implementation {
    components MainC;
    components Node;
    //components new HashmapC(int,100) as HashC;
    components new ListC(neighbor,100) as List;
    components new ListC(pack,100) as List2;
    components new AMReceiverC(AM_PACK) as GeneralReceive;
    components new TimerMilliC() as myTimerC; //create a new timer with alias “myTimerC”
    components new TimerMilliC() as lspTimer;
    components new TimerMilliC() as updateNeighbors;
    components new TimerMilliC() as pingTimer;

    components RandomC as Random;

    Node -> MainC.Boot;
    Node.Random -> Random;
    Node.lspTimer -> lspTimer;
    Node.Receive -> GeneralReceive;
    
    //Node.Hash -> HashC;
    Node.NeighborList -> List;
    Node.SeenLspPackList->List2;
    
    Node.Timer1 -> myTimerC; //Wire the interface to the component
    Node.updateNeighbors-> updateNeighbors;
    Node.pingTimer->pingTimer;
    
    components ActiveMessageC;
    Node.AMControl -> ActiveMessageC;
    
    components new SimpleSendC(AM_PACK);
    Node.Sender -> SimpleSendC;
    
    components CommandHandlerC;
    Node.CommandHandler -> CommandHandlerC;
     //add component for seenPacketList
    components new ListC(pack, 64) as PacketListC;
    Node.SeenPackList -> PacketListC; //connects seenPacketList with component ListC
}