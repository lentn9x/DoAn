

#ifndef _GEOBASIC_H_
#define _GEOBASIC_H_
class IMobility;

#define MIXIM_INET
#include <list>

#include <omnetpp.h>

#include "MiXiMDefs.h"
#include "BaseNetwLayer.h"
#include "SimpleAddress.h"
#include "Coord.h"
#include "IMobility.h"
#include <cmsgpar.h>
#include <vector>
#include "GeoNode.h"
#include "NeighborUpdatePkt_m.h"
#include "GreedyPkt_m.h"
#include "SensorApplLayer.h"
/**
 * @brief A simple flooding protocol
 *
 * This implementation uses plain flooding, i.e. it "remembers"
 * (stores) already broadcasted messages in a list and does not
 * rebroadcast them again, if it gets another copy of that message.
 *
 * The maximum number of entires for that list can be defined in the
 * .ini file (@ref bcMaxEntries) as well as the time after which an entry
 * is deleted (@ref bcDelTime).
 *
 * If you prefere a memory-less version you can comment out the 
 * @verbatim #define PLAINFLOODING @endverbatim
 *
 * @ingroup netwLayer
 * @author Daniel Willkomm
 *
 * ported to Mixim 2.0 by Theodoros Kapourniotis
 **/
class MIXIM_API GeoBasic : public BaseNetwLayer
{
protected:
        GeoNode myNode;
        double tx_interval;
        cMessage *timerMsg;
        int smCounter;
        cPacket *msg;
        IMobility *myMobi, *destMobi;
        Coord myCor;
        std::vector<GeoNode> nbTable;
        std::vector<GeoNode> nbTableNext;
        int nbUpdateCnt;
        enum messagesTypes {
            UNKNOWN=0,
            DATA
        };

public:
        GeoBasic(): BaseNetwLayer(){}

    /** @brief Initialization of omnetpp.ini parameters*/
    virtual void initialize(int);
    virtual void finish();    

protected:

    /** @brief Handle messages from upper layer */
    virtual void handleUpperMsg(cMessage *);

    /** @brief Handle messages from lower layer */
    virtual void handleLowerMsg(cMessage *);

    /** @brief we have no self messages */
    virtual void handleSelfMsg(cMessage* msg);
    
    virtual void updateNbTable(GeoNode nbNode);
    virtual GeoNode* findNextNode(GeoNode destNode);
    virtual GeoNode* findPoint(GeoNode a,GeoNode b);
    virtual GeoNode* dis_Point(GeoNode a,GeoNode b);
    virtual netwpkt_ptr_t encapsMsg(cPacket*);

};

#endif
