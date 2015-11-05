/* -*- mode:c++ -*- ********************************************************
 * file:        Flood.cc
 *
 * author:      Daniel Willkomm
 *
 * copyright:   (C) 2004 Telecommunication Networks Group (TKN) at
 *              Technische Universitaet Berlin, Germany.
 *
 *              This program is free software; you can redistribute it 
 *              and/or modify it under the terms of the GNU General Public 
 *              License as published by the Free Software Foundation; either
 *              version 2 of the License, or (at your option) any later 
 *              version.
 *              For further information see file COPYING 
 *              in the top level directory
 *
 ***************************************************************************
 * part of:     framework implementation developed by tkn
 * description: a simple flooding protocol
 *              the user can decide whether to use plain flooding or not
 **************************************************************************/

#include <BaseModule.h>
#include <cenvir.h>
#include <cmessage.h>
#include <cmodule.h>
#include <cnamedobject.h>
#include <cobjectfactory.h>
#include <cownedobject.h>
#include <cpar.h>
#include <cregistrationlist.h>
#include <csimulation.h>
#include <GeoBasic.h>
#include <INETDefs.h>
#include <regmacros.h>
#include <simkerneldefs.h>
#include <simtime.h>
#include <simutil.h>
#include <cstring>
#include <iostream>

using std::endl;

Define_Module(GeoBasic);

void GeoBasic::initialize(int stage) {
    BaseNetwLayer::initialize(stage);
    tx_interval = par("Tx_interval");
    smCounter = 0;
    nbUpdateCnt = 0;
    //my current location
    myMobi = dynamic_cast<IMobility*>(getParentModule()->getSubmodule(
            "mobility"));
    myCor = myMobi->getCurrentPosition();
    myNode.locationX = myCor.x;
    myNode.locationY = myCor.y;
    myNode.nodeID = getNode()->getId();
    if (stage == 0) {
        timerMsg = new cMessage("timer message");
    }
    if (stage == 1) {
        scheduleAt(simTime() + tx_interval, timerMsg);
    }
    EV << "Test 2 node bat ki" << endl;
    GeoNode* l1 = new GeoNode();
    GeoNode* l2 = new GeoNode();
    GeoNode* temp = new GeoNode();
    l1->locationX = 100;l1->locationY = 200;
    l2->locationX = 300;l2->locationY = 400;
    temp = findPoint(*l1,*l2);
    EV << temp->locationX << temp->locationY << endl;
}
void GeoBasic::handleSelfMsg(cMessage *msg) {
    if (strcmp(msg->getName(), "timer message") == 0) {
//        ev<<"receive self message"<<endl;
        if (nbUpdateCnt < 5)
            scheduleAt(simTime() + tx_interval, msg); //schedule self-message
//create and send out broadcast Msg
        NeighborUpdatePkt* pkt = new NeighborUpdatePkt("NbBroadcast");
        pkt->setBitLength(headerLength);
        pkt->setDestAddr(LAddress::L3BROADCAST);
        pkt->setNodeInfo(myNode);
        setDownControlInfo(pkt, LAddress::L2BROADCAST);
        sendDown(pkt);
        nbUpdateCnt++;
    }
}
void GeoBasic::finish() {
    //if (par("stats")) {
    for (unsigned int i = 0; i < nbTable.size(); i++) {
        recordScalar("ID", nbTable[i].nodeID);
        recordScalar("locationX", nbTable[i].locationX);
        recordScalar("locationY", nbTable[i].locationY);
    }
    // }
    BaseNetwLayer::finish();

}
void GeoBasic::handleUpperMsg(cMessage* msg) {
//    EV << "value of isSource node" << par("isSourceNode").boolValue() <<endl;
    if (par("isSourceNode").boolValue()
            && msg->getKind() == SensorApplLayer::DATA_MESSAGE) {
        //       EV <<" I am a source code " << endl;
        //       assert(dynamic_cast<cPacket*> (msg));
        GreedyPkt* greedyPkt;
        greedyPkt = static_cast<GreedyPkt*>(encapsMsg(
                static_cast<cPacket*>(msg)));
        greedyPkt->setName("GreedyPkt");
        GeoNode *destNode = new GeoNode();
        destNode->nodeID = par("destId").longValue();
        destNode->locationX = par("destX").doubleValue();
        destNode->locationY = par("destY").doubleValue();
        greedyPkt->setDestNode(*destNode);
        greedyPkt->setInterNode(*findNextNode(greedyPkt->getDestNode()));
        EV << "my node ID" << myNode.nodeID << endl;
        EV << "next node ID" << greedyPkt->getInterNode().nodeID << endl;
        greedyPkt->setBitLength(headerLength);
        greedyPkt->setDestAddr(LAddress::L3BROADCAST);
        //    setDownControlInfo(greedyPkt, LAddress::L2BROADCAST);
        sendDown(greedyPkt);
        EV << "SOURCE: send a message" << endl;
    } else {
        delete (msg);
    }
}
void GeoBasic::updateNbTable(GeoNode nbNode) {
    bool updateTable = true;
    if (nbTable.size() == 0) {
        nbTable.push_back(nbNode);
    } else {
        for (unsigned int i = 0; i < nbTable.size(); i++) {
            if (nbTable[i] == nbNode) {
                updateTable = false;
                break;
            }
        }
        if (updateTable)
            nbTable.push_back(nbNode);
    }
}
GeoNode* GeoBasic::findPoint(GeoNode i1,GeoNode i2){

    int a1,b1,a2,b2;
    int x1,x2,x,y1,y2,y,r,resx,resy;
    double a,b,c,aa,bb,delta;
    r = i1.radius;
    aa = (b1-b2)/(a2-a1);
    bb = (a2*a2+b2*b2-a1*a1-b1*b1)/(2*(a2-a1))-a1;
    a = aa*aa+1;
    b = 2*(aa*bb-b1);
    c = bb*bb+b1*b1-r*r;
    delta = b*b-4*a*c;
    y1 = (-b-sqrt(delta))/(2*a);
    x1 = aa*y1 + bb+a1;
    y2 = (-b+sqrt(delta))/(2*a);
    x2 = aa*y1 + bb+a1;
    x = i1.locationX;
    y = i1.locationY;
    GeoNode* point1 = new GeoNode();
    GeoNode* point2 = new GeoNode();

    point1->locationX = x1;point1->locationY = y1;
    point2->locationX = x2;point2->locationY = y2;
    //cung 1 goc phan tu
    if ((x1>x) && (y1>y) && (x2>x) && (y2>y)) {
        if (y1>y2) {
            resx = x1;resy=y1;
        } else {
            resx = x2;resy=y2;
        }
    } else if ((x1<x) && (y1>y) && (x2<x) && (y2>y)) {
        if (y1 < y2) {
            resx = x1;resy=y1;
        } else {
            resx = x2;resy=y2;
        }
    } else if ((x1<x) && (y1<y) && (x2<x) && (y2<y)) {
        if (y1 < y2) {
            resx = x1;resy=y1;
        } else {
            resx = x2;resy=y2;
        }
    } else if ((x1>x) && (y1<y) && (x2>x) && (y2<y)) {
        if (y1 > y2) {
            resx = x1;resy=y1;
        } else {
            resx = x2;resy=y2;
        }
    }
    // cac goc phan tu lien tiep
    else if (((x1-x)*(x2-x)<0) && ((y1-y)*(y2-y)>0)) {
        if (x1 < x2) {
            resx = x1;resy=y1;
        } else {
            resx = x2;resy=y2;
        }
    } else if (((x1-x)*(x2-x)>0) && ((y1-y)*(y2-y)<0)) {
        if (y1 < y2) {
            resx = x1;resy = y1;
        }  else {
            resx = x2;resy = y2;
        }
    } else if (((x1-x)*(x2-x)<0) && ((y1-y)*(y2-y)>0)) {
        if (x1 > x2) {
            resx = x1;resy = y1;
        }  else {
            resx = x2;resy = y2;
        }
    } else if (((x1-x)*(x2-x)>0) && ((y1-y)*(y2-y)>0)) {
        if (y1 > y2) {
            resx = x1;resy = y1;
        } else {
            resx = x2;resy = y2;
        }
    }
    //cac goc phan tu doi nhau
    else if ((x1<x) && (y1<y) && (x2>x) && (y2>y)) {
        if ((x2-x) > (y2-y)) {
            resx = x2;resy = y2;
        } else {
            resx = x1;resy = y1;
        }
    } else if ((x1>x) && (y1>y) && (x2<x) && (y2<y)) {
        if ((x1-x) > (y1-y)) {
            resx = x1;resy = y1;
        } else {
            resx = x2;resy = y2;
        }
    } else if ((x1<x) && (y1>y) && (x2>x) && (y2<y)) {
        if ((x1-x) < (y1-y)){
            resx = x1;resy = y1;
        } else {
            resx = x2;resy = y2;
        }
    } else if ((x1>x) && (y1<y) && (x2<x) && (y2>y)) {
        if ((x2-x) < (y2-y)){
            resx = x2;resy = y2;
        } else {
            resx = x1;resy = y1;
        }
    }
    GeoNode* res = new GeoNode();
    res->locationX = resx;res->locationY = resy;
    return res;
}
GeoNode* GeoBasic::initCoverageNode(bool myNode,GreedyPkt* greedyPkt) {
    double dist = myNode.distance(destNode);
    GeoNode* nextNode = new GeoNode();
    GeoNode* nextNode2 = new GeoNode();
    GeoNode* point1 = new GeoNode();
    GeoNode* point2 = new GeoNode();
    nextNode = &myNode;
    double mindis = 2*myNode.radius;
    GeoNode* minNode = new GeoNode();
    return false;
    bool flag = true;
    for (unsigned int i = 0; i < nbTable.size(); i++) {
        if (flag = false) break;
        {
            if (myNode.distance(nbTable[i]) < 2*myNode.radius) {
                nextNode = &nbTable[i];
                point1 = findPoint(myNode,nextNode);
                for (unsigned int j = 0; j < nbTableNext.size(); j++){
                    if (nextNode->distance(nbTableNext[j]) < 2*nextNode->radius) {
                        nextNode2 = &nbTableNext[j];
                        point2 = findPoint(nextNode,nextNode2);
                        if (point2->distance(myNode) <= myNode.radius) {
                            break;
                        } else {
                         if (point2->distance(myNode) > myNode.radius){
                            if (point2->distance(myNode) < mindis) {
                                mindis = point2->distance(myNode);
                                minNode = nextNode2;
                            }
                         }
                        }
                    }
                    if  (mindis < 2*myNode.radius) {
                        greedyPkt->initNode = myNode;
                        greedyPkt->initNode2 = nextNode;
                        greedyPkt->initNode3 = minNode;
                        flag = false;
                    }
                }
            }
        }
    }
    if (mindis < 2*myNode.radius) return true;
}
GeoNode* GeoBasic::findNextNode(GeoNode myNode){
//    double dist = myNode.distance(destNode);
    GeoNode* nextNode = new GeoNode();
    GeoNode* point1= new GeoNode();
    GeoNode* minNode = new GeoNode();
    nextNode = &myNode;
    double mindis = 2*myNode.radius;
    for (unsigned int i = 0; i < nbTable.size(); i++) {
           {
               if (myNode.distance(nbTable[i]) < 2*myNode.radius) {
                   nextNode = &nbTable[i];
                   point1 = findPoint(myNode,nextNode);
                           if (point1->distance(myNode) <= myNode.radius) {
                               break;
                           } else {
                            if (point1->distance(myNode) > myNode.radius){
                               if (point1->distance(myNode) < mindis) {
                                   mindis = point1->distance(myNode);
                                   minNode = nextNode;
                               }
                            }
                   }
               }
           }
       }
}
void GeoBasic::handleLowerMsg(cMessage* msg) {
    if (strcmp(msg->getName(), "NbBroadcast") == 0) {
        NeighborUpdatePkt* netwMsg = check_and_cast<NeighborUpdatePkt*>(msg);
        if (LAddress::isL3Broadcast(netwMsg->getDestAddr())) {
            updateNbTable(netwMsg->getNodeInfo());
        }
        EV << "node Id: " << myNode.nodeID << endl;
        for (unsigned int i = 0; i < nbTable.size(); i++) {
            EV << nbTable[i].nodeID << endl;
        }
        delete (netwMsg);
        EV << "update cac node lan can" << endl;
    } else if (strcmp(msg->getName(), "GreedyPkt") == 0) {
        EV << "lay goi tin greedy" << endl;
        GreedyPkt* greedyPkt = check_and_cast<GreedyPkt*>(msg);
        EV << "myNode ID" << myNode.nodeID << endl;
        EV << "interNode ID" << greedyPkt->getInterNode().nodeID << endl;
        EV << "destNode ID" << greedyPkt->getDestNode().nodeID << endl;
        if (greedyPkt->getDestNode().nodeID != myNode.nodeID
                && greedyPkt->getInterNode().nodeID != myNode.nodeID) {
            //  EV <<"received a packet"<<endl;
            delete (greedyPkt);
            endSimulation();
        } else {
            if (greedyPkt->getDestNode().nodeID == myNode.nodeID) {
                EV << "DESTINATION: da den dich" << endl;
                delete (greedyPkt);
                endSimulation();
            } else {
                //             EV<< "forwards a packet" <<endl;
                if (findNextNode(greedyPkt->getDestNode()) == &myNode) {
                    delete (greedyPkt);
                    EV << "Bi mac ket: goi tin bi mac ket o day !" << endl;
                } else {
                    cObject * const pCtrlInfo = greedyPkt->removeControlInfo();
                    if (pCtrlInfo != NULL)
                        delete pCtrlInfo;
                    //     setDownControlInfo(greedyPkt, LAddress::L2BROADCAST);
                    greedyPkt->setInterNode(
                            *findNextNode(greedyPkt->getDestNode()));
                    greedyPkt->setDestAddr(LAddress::L3BROADCAST);
                    setDownControlInfo(greedyPkt, LAddress::L2BROADCAST);
                    sendDown(greedyPkt);
                    EV << "INTERMEDIATE: quay lai goi tin truoc" << endl;
                }
            }
        }
    }
}
GeoBasic::netwpkt_ptr_t GeoBasic::encapsMsg(cPacket *appPkt) {
    cPacket* msg = static_cast<cPacket*>(appPkt);
    GreedyPkt* pkt = new GreedyPkt(msg->getName(), DATA);
    cObject* cInfo = msg->removeControlInfo();

    ASSERT(cInfo);
    pkt->setBitLength(headerLength);
    pkt->setSrcAddr(myNetwAddr);
    pkt->setDestAddr(LAddress::L3BROADCAST);

    setDownControlInfo(pkt, LAddress::L2BROADCAST);
    //encapsulate the application packet
    pkt->encapsulate(msg);

    // clean-up
    delete cInfo;

    return pkt;
}
