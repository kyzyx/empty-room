#include "pairwise.h"
#include "findplanes.h"

using namespace pcl;
using namespace Eigen;
using namespace std;

const double ANGLETHRESHOLD = M_PI/12;
const double MATCHANGLETHRESHOLD = M_PI/6;

void filterManhattan(
        Vector3d p1, Vector3d p2,
        PointCloud<PointXYZ>::ConstPtr cloud,
        vector<Vector4d>& planes,
        vector<int>& ids)
{
    vector<Vector3d> axes;
    axes.push_back(p1);
    axes.push_back(p2);
    axes.push_back(p1.cross(p2).normalized());
    for (int i = 0; i < planes.size(); ++i) {
        for (int j = 0; j < 3; ++j) {
            if (abs(planes[i].head(3).dot(axes[j])) > cos(ANGLETHRESHOLD)) {
                if (planes[i].head(3).dot(axes[j]) > 0) {
                    planes[i].head(3) = axes[j];
                } else {
                    planes[i].head(3) = -axes[j];
                }
                break;
            }
        }
    }
    vector<double> offsets(planes.size(), 0);
    vector<int> counts(planes.size(), 0);
    for (int i = 0; i < cloud->size(); ++i) {
        if (ids[i] >= 0) {
            counts[ids[i]]++;
            offsets[ids[i]] -= Vector3d(cloud->at(i).x, cloud->at(i).y, cloud->at(i).z).dot(planes[ids[i]].head(3));
        }
    }
    for (int i = 0; i < planes.size(); ++i) {
        planes[i](3) = offsets[i]/counts[i];
    }
}


int findPlaneCorrespondences(
        PointCloud<PointXYZ>::ConstPtr src,
        PointCloud<PointXYZ>::ConstPtr tgt,
        vector<Vector4d>& srcplanes, vector<int>& srcids,
        vector<Vector4d>& tgtplanes, vector<int>& tgtids,
        vector<int>& planecorrespondences)
{
    planecorrespondences.resize(srcplanes.size(), -1);
    int numcorrespondences = 0;

    // Find all possible correspondences
    for (int i = 0; i < srcplanes.size(); ++i) {
        for (int j = 0; j < tgtplanes.size(); ++j) {
            double cosa = srcplanes[i].head(3).dot(tgtplanes[j].head(3));
            if (cosa > cos(MATCHANGLETHRESHOLD)) {
                if (planecorrespondences[i] != -1) {
                    if (abs(srcplanes[i](3) - tgtplanes[j](3)) < abs(srcplanes[i](3) - tgtplanes[planecorrespondences[i]](3))) {
                        planecorrespondences[i] = j;
                    }
                } else {
                    planecorrespondences[i] = j;
                }
            }
        }
        if (planecorrespondences[i] != -1) numcorrespondences++;
    }
    // Find duplicate matches and eliminate less plausible one
    for (int i = 0; i < srcplanes.size(); ++i) {
        if (planecorrespondences[i] < 0) continue;
        for (int j = i+1; j < srcplanes.size(); ++j) {
            if (planecorrespondences[i] == planecorrespondences[j]) {
                int t = planecorrespondences[i];
                if (abs(srcplanes[i](3) - tgtplanes[t](3)) > abs(srcplanes[j](3) - tgtplanes[t](3))) {
                    planecorrespondences[i] = -1;
                    break;
                } else {
                    planecorrespondences[j] = -1;
                }
            }
        }
    }

    // Group correspondences based on Manhattan coordinate system
    // TODO: Weight by plane area?
    vector<int> compatible(planecorrespondences.size(), -1);
    vector<int> compatiblecounts;
    int n = 0;
    for (int i = 0; i < planecorrespondences.size(); ++i) {
        if (compatible[i] == -1) {
            compatible[i] = n;
            compatiblecounts.push_back(1);
            for (int j = i+1; j < planecorrespondences.size(); ++j) {
                if (compatible[j] == -1 && srcplanes[i].head(3).dot(srcplanes[j].head(3)) < sin(ANGLETHRESHOLD)) {
                    compatible[j] = n;
                    ++compatiblecounts.back();
                }
            }
            ++n;
        }
    }

    // Select Manhattan coordinate system with most planes
    int maxcount = 0;
    n = -1;
    for (int i = 0; i < compatiblecounts.size(); ++i) {
        if (compatiblecounts[i] > maxcount) {
            n = i;
            maxcount = compatiblecounts[i];
        }
    }

    // Remove correspondences inconsistent with Manhattan coordinate system
    for (int i = 0; i < planecorrespondences.size(); ++i) {
        if (compatible[i] != n) {
            planecorrespondences[i] = -1;
        }
    }

    // Isolate at most one plane in each direction to match
    for (int i = 0; i < planecorrespondences.size(); ++i) {
        if (planecorrespondences[i] > -1) {
            int best = i;
            int orig = planecorrespondences[i];
            double rearmost = srcplanes[i](3);
            planecorrespondences[i] = -1;
            for (int j = i+1; j < planecorrespondences.size(); ++j) {
                if (planecorrespondences[j] > -1 && srcplanes[i].head(3).dot(srcplanes[j].head(3)) > cos(ANGLETHRESHOLD)) {
                    if (srcplanes[j](3) > rearmost) {
                        rearmost = srcplanes[j](3);
                        best = j;
                        orig = planecorrespondences[j];
                    }
                    planecorrespondences[j] = -1;
                }
            }
            planecorrespondences[best] = orig;
        }
    }

    // Final count of correspondences
    numcorrespondences = 0;
    vector<int> planeids;
    for (int i = 0; i < planecorrespondences.size(); ++i) {
        if (planecorrespondences[i] > -1) {
            planeids.push_back(i);
            ++numcorrespondences;
        }
    }

    // Force perpendicularity
    if (numcorrespondences > 1) {
        Vector3d p1 = srcplanes[planeids[0]].head(3);
        Vector3d pp = srcplanes[planeids[1]].head(3);
        Vector3d p2 = p1.cross(pp);
        p2.normalize();
        filterManhattan(p1, p2, src, srcplanes, srcids);
        p1 = tgtplanes[planecorrespondences[planeids[0]]].head(3);
        pp = tgtplanes[planecorrespondences[planeids[1]]].head(3);
        p2 = p1.cross(pp);
        p2.normalize();
        filterManhattan(p1, p2, tgt, tgtplanes, tgtids);
    }

    return numcorrespondences;
}
