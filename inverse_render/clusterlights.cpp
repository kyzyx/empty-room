#include "clusterlights.h"
#include "material.h"
#include <bitset>
#include <deque>

#define UNLABELLED_LIGHT 127

using namespace std;

// Marks all lights with a 1
void labelLights(Mesh& m, double hdrthreshold) {
    for (int i = 0; i < m.getMesh()->NVertices(); ++i) {
        Material mat;
        double total = 0;
        for (int k = 0; k < m.samples[i].size(); ++k) {
            Sample& s = m.samples[i][k];
            if (s.confidence > 0) {
                mat += Material(s.r,s.g,s.b)*abs(s.dA)*s.confidence;
                total += s.dA*s.confidence;
            }
        }
        if (total > 0) {
            mat /= total;
        }
        /*int count = 0;
        for (int j = 0; j < m.samples[i].size(); ++j) {
            if (m.samples[i][j].label == 1) ++count;
        }
        if (count > 0 && count + 2 >= m.samples[i].size() && m.samples[i].size() > 1) {*/
        if (mat.r > hdrthreshold || mat.g > hdrthreshold || mat.b > hdrthreshold) {
            m.labels[i] = UNLABELLED_LIGHT;
        } else {
            m.labels[i] = 0;
        }
    }
}
int bfsLabel(Mesh& m, int start, int allowed, int label) {
    vector<bool> visited(m.getMesh()->NVertices(),false);
    deque<int> q;
    q.push_back(start);
    visited[start] = true;
    m.labels[start] = label;
    int nchanged = 1;
    while(!q.empty()) {
        int n = q.front(); q.pop_front();
        R3MeshVertex* v = m.getMesh()->Vertex(n);
        for (int i = 0; i < m.getMesh()->VertexValence(v); ++i) {
            int u = m.getMesh()->VertexID(m.getMesh()->VertexOnVertex(v,i));
            if (m.labels[u] == allowed && !visited[u]) {
                q.push_back(u);
                m.labels[u] = label;
                visited[u] = true;
                nchanged++;
            }
        }
    }
    return nchanged;
}
int clusterLights(Mesh& m, double hdrthreshold, int minLightSize) {
    int lightn = 1;
    labelLights(m, hdrthreshold);
    for (int i = 0; i < m.getMesh()->NVertices(); ++i) {
        if (m.labels[i] == UNLABELLED_LIGHT) {
            int count = bfsLabel(m, i, UNLABELLED_LIGHT, lightn);
            if (count < minLightSize) bfsLabel(m, i, lightn, 0);
            else lightn++;
        }
    }
    return lightn-1;
}
