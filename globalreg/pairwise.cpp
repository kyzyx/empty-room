#include "pairwise.h"
#include "findplanes.h"
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>

using namespace pcl;
using namespace Eigen;
using namespace std;

const double ANGLETHRESHOLD = cos(M_PI/9);

Matrix4d overlapPlanes(Vector4d src, Vector4d tgt) {
    // Construct rotation
    Quaterniond q;
    q.setFromTwoVectors(src.head(3), tgt.head(3));
    // Construct points on src and tgt
    Vector3d psrc = -src.head(3)*src(3);
    Vector3d ptgt = -tgt.head(3)*tgt(3);

    double d = (q*psrc - ptgt).dot(tgt.head(3));

    Translation3d trans(-d*tgt.head(3));
    return (trans*q).matrix();
}

Matrix4d alignEdgeToEdge(
        PointCloud<PointXYZ>::ConstPtr src,
        PointCloud<PointXYZ>::ConstPtr tgt,
        vector<Vector4d>& srcplanes, vector<int>& srcids,
        vector<Vector4d>& tgtplanes, vector<int>& tgtids,
        vector<int>& planecorrespondences)
{
    return Matrix4d::Identity();
}

Matrix4d alignCornerToCorner(
        PointCloud<PointXYZ>::ConstPtr src,
        PointCloud<PointXYZ>::ConstPtr tgt,
        vector<Vector4d>& srcplanes, vector<int>& srcids,
        vector<Vector4d>& tgtplanes, vector<int>& tgtids,
        vector<int>& planecorrespondences)
{
    return Matrix4d::Identity();
}

Matrix4d alignICP(
        PointCloud<PointXYZ>::ConstPtr src,
        PointCloud<PointXYZ>::ConstPtr tgt,
        vector<Vector4d>& srcplanes, vector<int>& srcids,
        vector<Vector4d>& tgtplanes, vector<int>& tgtids,
        vector<int>& planecorrespondences)
{
    PointCloud<PointXYZ>::Ptr s(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr t(new PointCloud<PointXYZ>);
    vector<int> tmp;
    removeNaNFromPointCloud(*src, *s, tmp);
    removeNaNFromPointCloud(*tgt, *t, tmp);

    IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    icp.setInputSource(s);
    icp.setInputTarget(t);
    icp.setMaximumIterations(1);
    icp.align(*s);
    return icp.getFinalTransformation().cast<double>();
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
    for (int i = 0; i < srcplanes.size(); ++i) {
        for (int j = 0; j < tgtplanes.size(); ++j) {
            double cosa = 0;
            for (int k = 0; k < 3; ++k) cosa += srcplanes[i](k)*tgtplanes[j](k);
            if (cosa > ANGLETHRESHOLD) {
                if (planecorrespondences[i] != -1) {
                    cerr << "WARNING: Multiple correspondences possible!" << endl;
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
    return numcorrespondences;
}

Matrix4d align(
        PointCloud<PointXYZ>::ConstPtr src,
        PointCloud<PointXYZ>::ConstPtr tgt,
        vector<Vector4d>& srcplanes, vector<int>& srcids,
        vector<Vector4d>& tgtplanes, vector<int>& tgtids)
{
    // Detect planes
    if (tgtids.size() != tgt->size()) {
        findPlanes(tgt, tgtplanes, tgtids);
    }
    if (srcids.size() != src->size()) {
        findPlanes(src, srcplanes, srcids);
    }

    // Find plane correspondences
    // TODO: What if multiple compatible planes?
    vector<int> planecorrespondences;
    int numcorrespondences = findPlaneCorrespondences(src, tgt, srcplanes, srcids, tgtplanes, tgtids, planecorrespondences);
    // Call appropriate alignment function
    switch (numcorrespondences) {
        case 0:
            return alignICP(src, tgt, srcplanes, srcids, tgtplanes, tgtids, planecorrespondences);
            break;
        case 1:
            return alignPlaneToPlane(src, tgt, srcplanes, srcids, tgtplanes, tgtids, planecorrespondences);
            break;
        case 2:
            return alignEdgeToEdge(src, tgt, srcplanes, srcids, tgtplanes, tgtids, planecorrespondences);
            break;
        case 3:
            return alignCornerToCorner(src, tgt, srcplanes, srcids, tgtplanes, tgtids, planecorrespondences);
            break;
        default:
            cerr << "Error! " << numcorrespondences << " correspondences found!" << endl;
    }
    return Matrix4d::Identity();
}

Vector4d transformPlane(Vector4d plane, Matrix4d transform)
{
    Vector4d dir = plane;
    dir(3) = 0;
    Vector4d p(-plane(3)*plane(0), -plane(3)*plane(1), -plane(3)*plane(2), 1);

    dir = transform*dir;
    p = transform*p;

    dir(3) = -dir.dot(p);
    return dir;
}
