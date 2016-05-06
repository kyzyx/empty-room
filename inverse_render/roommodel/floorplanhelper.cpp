#include "roommodel/floorplanhelper.h"

using namespace std;

int FloorplanHelper::closestWall(Eigen::Vector3f p, Eigen::Vector3f n) {
    for (int i = 0; i < wallsegments.size(); i++) {
        if (wallsegments[i].isCompatiblePoint(p, n, 0.1)) return i;
    }
    return -1;
}

Eigen::Vector3f FloorplanHelper::getWallPoint(int i, double coord, double height) const {
    if (i >= wallsegments.size()) i %= wallsegments.size();
    Eigen::Vector4f p;
    pair<double,double> x = wallsegments[i].getCoords(coord + wallsegments[i].start);
    p[0] = x.first;
    p[1] = floorplane + height;
    p[2] = x.second;
    p[3] = 1;
    p = world2floorplan.inverse()*p;
    return Eigen::Vector3f(p[0]/p[3], p[1]/p[3], p[2]/p[3]);
}

Eigen::Vector3f FloorplanHelper::getNormalizedWallEndpoint(int i, bool lo, double height) const {
    if (i >= wallsegments.size()) i %= wallsegments.size();
    double coord = (lo==forwards[i])?wallsegments[i].start:wallsegments[i].end;
    pair<double,double> x = wallsegments[i].getCoords(coord);
    Eigen::Vector3f p;
    p[0] = x.first;
    p[1] = height*ceilplane + (1-height)*floorplane;
    p[2] = x.second;
    return p;
}
void FloorplanHelper::getAsRoomModel(roommodel::RoomModel* rm) {
    rm->height = ceilplane - floorplane;
    rm->originalFloor = floorplane;
    rm->walls.clear();
    int start;
    for (start = 0; start < wallsegments.size(); ++start) {
        if (wallsegments[start].direction == 1 && wallsegments[start].norm > 0) {
            break;
        }
    }
    if (start == wallsegments.size()) {
        return;
    }
    rotate(wallsegments.begin(), wallsegments.begin()+start, wallsegments.end());
    if (wallsegments[1].coord != wallsegments[0].end) {
        reverse(wallsegments.begin(), wallsegments.end());
        rotate(wallsegments.begin(), wallsegments.end()-1, wallsegments.end());
    }

    for (int i = 0; i < wallsegments.size(); ++i) {
        roommodel::Wall wall;
        wall.length = wallsegments[i].length();
        wall.normal = wallsegments[i].norm;
        rm->walls.push_back(wall);
    }
    rm->wallMaterial.diffuse.r = 1;
    rm->wallMaterial.diffuse.g = 1;
    rm->wallMaterial.diffuse.b = 1;
    rm->floorMaterial.diffuse.r = 1;
    rm->floorMaterial.diffuse.g = 1;
    rm->floorMaterial.diffuse.b = 1;
    rm->ceilingMaterial.diffuse.r = 1;
    rm->ceilingMaterial.diffuse.g = 1;
    rm->ceilingMaterial.diffuse.b = 1;
}

void FloorplanHelper::loadFromRoomModel(roommodel::RoomModel* rm) {
    wallsegments.clear();
    floorplane = rm->originalFloor;
    ceilplane = floorplane + rm->height;
    R4Matrix m = rm->globaltransform;
    normalization << m[0][0], m[0][1], m[0][2], m[0][3],
                     m[1][0], m[1][1], m[1][2], m[1][3],
                     m[2][0], m[2][1], m[2][2], m[2][3],
                     m[3][0], m[3][1], m[3][2], m[3][3];
    Eigen::Matrix4f reup = Eigen::Matrix4f::Identity();
    reup(0,0) = 0;
    reup(0,2) = 1;
    reup(2,2) = 0;
    reup(2,0) = 1;
    world2floorplan = reup*normalization;

    double px = 0;
    double py = 0;
    for (int i = 0; i < rm->walls.size(); ++i) {
        int nn = rm->walls[i].normal;
        double l = rm->walls[i].length*nn;
        double cx = (i&1)?px-l:px;
        double cy = (i&1)?py:py+l;

        Segment s;
        s.direction = i&1?1:0;
        s.start = (i&1)?px:py;
        s.end = (i&1)?cx:cy;
        if (s.start > s.end) swap(s.start, s.end);
        s.coord = (i&1)?py:px;
        s.norm = nn;

        px = cx;
        py = cy;

        forwards.push_back(i&1);
        wallsegments.push_back(s);
    }
}
