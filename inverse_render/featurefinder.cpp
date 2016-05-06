#include "featurefinder.h"
#include <limits>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

Condition::Condition()
    : m_idx(-1), m_lbl(-1)
{
    float m = numeric_limits<float>::lowest();
    minp = Vector3f(m,m,m);
    m = numeric_limits<float>::max();
    maxp = Vector3f(m,m,m);
}
void FeatureFinder::computeIndices(
        FloorplanHelper& fph,
        MeshManager* mmgr,
        vector<int>& indices)
{
    vector<double> ddepth;
    double dwall = 0;
    int dwallcount = 0;
    for (int i = 0; i < mmgr->size(); i++) {
        Eigen::Vector3f p = mmgr->VertexPositionE(i);
        Eigen::Vector3f n = mmgr->VertexNormalE(i);
        Eigen::Vector4f p4(p(0), p(1), p(2), 1);
        Eigen::Vector4f n4(n(0), n(1), n(2), 0);
        p = (fph.world2floorplan*p4).head<3>();
        n = (fph.world2floorplan*n4).head<3>();
        int wallidx = fph.closestWall(p, n);
        if (condition(p, n, wallidx, mmgr->getLabel(i, MeshManager::TYPE_CHANNEL))) {
            indices.push_back(i);
            if (wallidx >= 0) {
                double d = 0;
                if (fph.wallsegments[wallidx].direction > 0) {
                    d = p(2) - fph.wallsegments[wallidx].coord;
                } else {
                    d = p(0) - fph.wallsegments[wallidx].coord;
                }
                if (fph.wallsegments[wallidx].norm < 0) d = -d;
                ddepth.push_back(d);
            }
        } else if (wallidx >= 0 && condition.getLabel() == mmgr->getLabel(i, MeshManager::TYPE_CHANNEL))
        {
            if (condition.getWallIndex() < 0 || wallidx == condition.getWallIndex()) {
                double d = 0;
                if (fph.wallsegments[wallidx].direction > 0) {
                    d = p(2) - fph.wallsegments[wallidx].coord;
                } else {
                    d = p(0) - fph.wallsegments[wallidx].coord;
                }
                if (fph.wallsegments[wallidx].norm < 0) d = -d;
                dwall += d;
                dwallcount++;
            }
        }
    }
    if (ddepth.size()) {
        depth = accumulate(ddepth.begin(), ddepth.end(), 0.0) / ddepth.size();
    }
    if (dwallcount) {
        dwall /= dwallcount;
        depth -= dwall;
        cout << "Mean wall depth: " << dwall << ", feature depth " << depth << endl;
    }
}

void FeatureFinder::compute(
        FloorplanHelper& fph,
        InverseRender& ir,
        vector<SampleData>& alldata)
{
    MeshManager* mmgr = ir.getRenderManager()->getMeshManager();
    vector<int> indices;
    computeIndices(fph, mmgr, indices);
    vector<SampleData> data;
    for (int i = 0; i < indices.size(); i++) {
        data.push_back(alldata[indices[i]]);
    }
    mat = ir.computeAverageMaterial(data);
}
