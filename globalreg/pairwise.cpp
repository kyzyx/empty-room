#include "pairwise.h"
#include "findplanes.h"
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>

using namespace pcl;
using namespace Eigen;
using namespace std;

const double ANGLETHRESHOLD = M_PI/9;
const double EPSILON = 0.00001;

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

Matrix4d overlapEdge(Vector4d src1, Vector4d src2, Vector4d tgt1, Vector4d tgt2) {
    // Assumes that src1, src2, tgt1, tgt2 are in correspondence and perpendicular
    // First determine common plane
    Matrix4d t1 = overlapPlanes(src1, tgt1);
    // Determine in-plane angle to rotate
    Vector4d tsrc2 = transformPlane(src2, t1);
    Vector3d vs2 = tsrc2.head(3);
    Vector3d vt2 = tgt2.head(3);
    Vector3d cross = vs2.cross(vt2);
    double angle = vs2.dot(vt2);
    if (abs(angle - 1) < EPSILON) angle = 0;
    else angle = acos(angle);
    if (cross.dot(tgt1.head(3)) < 0) angle = -angle;
    Matrix4d t2 = Matrix4d::Identity();//AngleAxisd(angle, tgt1).matrix();
    t2.topLeftCorner(3,3) = AngleAxisd(angle, tgt1.head(3)).matrix();
    Vector4d ttsrc2 = transformPlane(src2, t2*t1);
    Vector3d x = (ttsrc2(3) - tgt2(3))*ttsrc2.head(3);
    Matrix4d t3 = Matrix4d::Identity();
    t3.topRightCorner(3,1) = x;
    return t3*t2*t1;
}

Matrix4d overlapCorner(
        Vector4d src1, Vector4d src2, Vector4d src3,
        Vector4d tgt1, Vector4d tgt2, Vector4d tgt3)
{
    Matrix4d t1 = overlapEdge(src1, src2, tgt1, tgt2);
    Vector4d ts3 = transformPlane(src3, t1);
    Vector3d x = (ts3(3) - tgt3(3))*ts3.head(3);
    Matrix4d t2 = Matrix4d::Identity();
    t2.topRightCorner(3,1) = x;
    return t2*t1;
}

Matrix4d alignCornerToCorner(
        PointCloud<PointXYZ>::ConstPtr src,
        PointCloud<PointXYZ>::ConstPtr tgt,
        vector<Vector4d>& srcplanes, vector<int>& srcids,
        vector<Vector4d>& tgtplanes, vector<int>& tgtids,
        vector<int>& planecorrespondences)
{

    vector<int> ids;
    for (int i = 0; i < planecorrespondences.size(); ++i) {
        if (planecorrespondences[i] > -1) ids.push_back(i);
    }
    return overlapCorner(
            srcplanes[ids[0]],
            srcplanes[ids[1]],
            srcplanes[ids[2]],
            tgtplanes[planecorrespondences[ids[0]]],
            tgtplanes[planecorrespondences[ids[1]]],
            tgtplanes[planecorrespondences[ids[2]]]);
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
    icp.setMaximumIterations(50);
    icp.align(*s);
    return icp.getFinalTransformation().cast<double>();
}

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
    for (int i = 0; i < srcplanes.size(); ++i) {
        for (int j = 0; j < tgtplanes.size(); ++j) {
            double cosa = srcplanes[i].head(3).dot(tgtplanes[j].head(3));
            if (cosa > cos(ANGLETHRESHOLD)) {
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
    int maxcount = 0;
    n = -1;
    for (int i = 0; i < compatiblecounts.size(); ++i) {
        if (compatiblecounts[i] > maxcount) {
            n = i;
            maxcount = compatiblecounts[i];
        }
    }
    numcorrespondences = 0;
    vector<int> planeids;
    for (int i = 0; i < planecorrespondences.size(); ++i) {
        if (compatible[i] != n) planecorrespondences[i] = -1;
        else if (planecorrespondences[i] >= 0) {
            planeids.push_back(i);
            numcorrespondences++;
        }
    }
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
            cout << "Aligning ICP" << endl;
            return alignICP(src, tgt, srcplanes, srcids, tgtplanes, tgtids, planecorrespondences);
            break;
        case 1:
            cout << "Aligning plane to plane" << endl;
            return alignPlaneToPlane(src, tgt, srcplanes, srcids, tgtplanes, tgtids, planecorrespondences);
            break;
        case 2:
            cout << "Aligning edge to edge" << endl;
            return alignEdgeToEdge(src, tgt, srcplanes, srcids, tgtplanes, tgtids, planecorrespondences);
            break;
        case 3:
            cout << "Aligning corner to corner" << endl;
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
    dir.head(3) = dir.head(3).normalized();
    p = transform*p;

    dir(3) = -dir.dot(p);
    return dir;
}
