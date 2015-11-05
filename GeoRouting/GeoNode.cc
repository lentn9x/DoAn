#include "GeoNode.h"

GeoNode::GeoNode() {
    nodeID = 0;
    locationX = 0;
    locationY= 0;
}

GeoNode::GeoNode(double a, double b) {
    nodeID = 0;
    locationX = a;
    locationY= b;
}

GeoNode::GeoNode(long x, double a, double b) {
    nodeID = x;
    locationX = a;
    locationY= b;
}

bool GeoNode::operator==(const GeoNode& b) {
//    return ((nodeID == b.nodeID) && (locationX == b.locationX) && (locationY== b.locationY));
    return (nodeID == b.nodeID);
}

bool GeoNode::equal(const GeoNode& b) {
//    return ((nodeID == b.nodeID) && (locationX == b.locationX) && (locationY== b.locationY));
    return (nodeID == b.nodeID);
}

double GeoNode::distance(GeoNode a){
    double d = sqrt(pow((locationX - a.locationX),2) + pow((locationY - a.locationY),2));
    return d;
}
//double GeoNode::dis_point(GeoNode x,GeoNode y){
//    double res = sqrt(pow((x.locationX - y.locationX),2) + pow((x.locationY - y.locationY),2));
//    return res;
//}
