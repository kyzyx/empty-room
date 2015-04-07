#include "clusterlights.h"
#include "material.h"
#include <bitset>
#include <deque>

#define UNLABELLED_LIGHT 127

using namespace std;

// Marks all lights with a 1
void labelLights(MeshManager& m, double hdrthreshold) {
    for (int i = 0; i < m.NVertices(); ++i) {
        Material mat = m.getVertexColor(i);
        if (mat.r > hdrthreshold || mat.g > hdrthreshold || mat.b > hdrthreshold) {
            m.setLabel(i, UNLABELLED_LIGHT);
        } else {
            m.setLabel(i, 0);
        }
    }
}
int bfsLabel(MeshManager& m, int start, int allowed, int label) {
    vector<bool> visited(m.NVertices(),false);
    deque<int> q;
    q.push_back(start);
    visited[start] = true;
    m.setLabel(start, label);
    int nchanged = 1;
    while(!q.empty()) {
        int n = q.front(); q.pop_front();
        R3MeshVertex* v = m.getMesh()->Vertex(n);
        for (int i = 0; i < m.getMesh()->VertexValence(v); ++i) {
            int u = m.getMesh()->VertexID(m.getMesh()->VertexOnVertex(v,i));
            if (m.getLabel(u) == allowed && !visited[u]) {
                q.push_back(u);
                m.setLabel(u, label);
                visited[u] = true;
                nchanged++;
            }
        }
    }
    return nchanged;
}
int clusterLights(MeshManager& m, double hdrthreshold, int minLightSize) {
    int lightn = 1;
    labelLights(m, hdrthreshold);
    for (int i = 0; i < m.NVertices(); ++i) {
        if (m.getLabel(i) == UNLABELLED_LIGHT) {
            int count = bfsLabel(m, i, UNLABELLED_LIGHT, lightn);
            if (count < minLightSize) bfsLabel(m, i, lightn, 0);
            else lightn++;
        }
    }
    return lightn-1;
}
