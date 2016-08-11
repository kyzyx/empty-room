#ifndef _FEATUREFINDER_H
#define _FEATUREFINDER_H
#include <Eigen/Core>
#include "solver.h"
#include "roommodel/floorplanhelper.h"

class Condition {
    public:
        Condition();
        bool operator()(Eigen::Vector3f p, Eigen::Vector3f n, int idx, int lbl) {
            if (m_idx >= 0 && idx != m_idx) return false;
            if (m_lbl >= 0 && lbl != m_lbl) return false;
            for (int i = 0; i < 3; i++) {
                if (p[i] < minp[i]) {
                    return false;
                }
                if (p[i] > maxp[i]) {
                    return false;
                }
            }
            return true;
        }

        void setMin(int n, float v) { minp[n] = v; }
        void setMax(int n, float v) { maxp[n] = v; }
        void setWallIndex(int idx) { m_idx = idx; }
        int getWallIndex() const { return m_idx; }
        void setLabel(int lbl) { m_lbl = lbl; }
        int getLabel() const { return m_lbl; }
    private:
        Eigen::Vector3f minp, maxp;
        int m_idx, m_lbl;
};

class FeatureFinder {
    public:
        FeatureFinder() {;}

        void compute(
                FloorplanHelper& fph,
                InverseRender& ir,
                std::vector<SampleData>& alldata);
        void computeIndices(
                FloorplanHelper& fph,
                MeshManager* mmgr,
                std::vector<int>& indices);

        Material getMaterial() const { return mat; }
        double getDepth() const { return depth; }

        Condition condition;
    protected:
        double depth;
        Material mat;
};

#endif
