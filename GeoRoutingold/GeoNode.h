

#ifndef GEONODE_H_
#define GEONODE_H_
#define MIXIM_INET
#include <math.h>
#include <algorithm>

class GeoNode {
public:

    long nodeID;
    double locationX,locationY;
    double radius;
    GeoNode();
    GeoNode(double a, double b);
    GeoNode(long x, double a, double b);
    bool operator==(const GeoNode& b);
    bool equal(const GeoNode& b);
    double distance(GeoNode a);

};

#endif /* GeoNode_H_ */
