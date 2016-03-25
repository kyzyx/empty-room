#include "cubemap.h"
#include "sh.h"
#include <cmath>

using namespace std;

void computeEnvmapAdjacencies(vector<pair<int, int> >& adj, int res) {
    // Cube faces
    for (int i = 0; i < res; i++) {
        for (int j = 0; j < res; j++) {
            for (int k = 0; k < 6; k++) {
                int idx = k*res*res + i*res + j;
                int left = k*res*res + i*res + j - 1;
                int up = k*res*res + (i-1)*res + j;
                if (i) adj.push_back(make_pair(idx, up));
                if (j) adj.push_back(make_pair(idx, left));
            }
        }
    }
    // Four vertical edges
    for (int i = 0; i < res; i++) {
        for (int j = 0; j < 4; j++) {
            int idx = j*res*res + (i+1)*res - 1;
            int next = ((j+1)%4)*res*res + i*res;
            adj.push_back(make_pair(idx, next));
        }
    }
    // Four top edges
    for (int i = 0; i < res; i++) {
        int idx = 0*res*res + i;
        int frontedge = (5*res - 1)*res + i;
        adj.push_back(make_pair(idx, frontedge));
        idx = 1*res*res + i;
        int rightedge = 5*res*res - i*res - 1;
        adj.push_back(make_pair(idx, rightedge));
        idx = 2*res*res + i;
        int backedge = (4*res + 1)*res - i - 1;
        adj.push_back(make_pair(idx, backedge));
        idx = 3*res*res + i;
        int leftedge = 4*res*res + i*res;
        adj.push_back(make_pair(idx, leftedge));
    }
    // Four bottom edges
    for (int i = 0; i < res; i++) {
        int idx = (1*res - 1)*res + i;
        int frontedge = 5*res*res + i;
        adj.push_back(make_pair(idx, frontedge));
        idx = (2*res - 1)*res + i;
        int rightedge = 5*res*res + (i+1)*res - 1;
        adj.push_back(make_pair(idx, rightedge));
        idx = (3*res - 1)*res + i;
        int backedge = 6*res*res - i - 1;
        adj.push_back(make_pair(idx, backedge));
        idx = (4*res - 1)*res + i;
        int leftedge = 6*res*res - (i+1)*res;
        adj.push_back(make_pair(idx, leftedge));
    }
}

const double s3 = sqrt(3);

int getEnvmapCell(double x, double y, double z, int res) {
    double xx = abs(x); double yy = abs(y); double zz = abs(z);
    int n;
    double u, v;
    if (xx >= yy && xx >= zz) {
        n = x>0?0:2;
        u = x>0?-y:y;
        v = z;
    } else if (yy >= xx && yy >= zz) {
        n = y>0?3:1;
        u = y>0?x:-x;
        v = z;
    } else {
        n = z>0?5:4;
        u = -y;
        v = z>0?-x:x;
    }
    int uu = (u*s3*res + res)/2;
    int vv = (v*s3*res + res)/2;
    if (uu >= res) uu = res-1;
    if (vv >= res) vv = res-1;
    if (vv < 0) vv = 0;
    if (uu < 0) uu = 0;
    return n*res*res + res*vv + uu;
}
