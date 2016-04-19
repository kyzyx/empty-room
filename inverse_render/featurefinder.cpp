#include "featurefinder.h"
#include <limits>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

Condition::Condition()
    : m_idx(-1), m_lbl(-1)
{
    float m = numeric_limits<float>::min();
    minp = Vector3f(m,m,m);
    m = numeric_limits<float>::max();
    maxp = Vector3f(m,m,m);
}
void FeatureFinder::compute(
        FloorplanHelper& fph,
        InverseRender& ir,
        vector<SampleData>& alldata)
{
    MeshManager* mmgr = ir.getRenderManager()->getMeshManager();
    vector<SampleData> data;
    vector<double> ddepth;
    for (int i = 0; i < mmgr->size(); i++) {
        Eigen::Vector3f p = mmgr->VertexPositionE(i);
        Eigen::Vector3f n = mmgr->VertexNormalE(i);
        Eigen::Vector4f p4(p(0), p(1), p(2), 1);
        Eigen::Vector4f n4(n(0), n(1), n(2), 0);
        p = (fph.world2floorplan*p4).head<3>();
        n = (fph.world2floorplan*n4).head<3>();
        int wallidx = fph.closestWall(p, n);
        if (condition(p, n, wallidx, mmgr->getLabel(i, MeshManager::TYPE_CHANNEL))) {
            cout << i << " " << p[0] << " " << p[1] << " " << p[2] << endl;
            data.push_back(alldata[i]);
            if (wallidx >= 0) {
                double d = 0;
                if (fph.wallsegments[wallidx].direction > 0) {
                    d = fph.wallsegments[wallidx].coord - p(2);
                } else {
                    d = p(0) - fph.wallsegments[wallidx].coord;
                }
                ddepth.push_back(d);
            }
        }
    }
    mat = ir.computeAverageMaterial(data);
    if (ddepth.size()) {
        depth = accumulate(ddepth.begin(), ddepth.end(), 0.0) / ddepth.size();
        if (depth < 0) depth = 0;
    }
}
