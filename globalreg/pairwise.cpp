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

AlignmentResult alignCornerToCorner(
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
    Matrix4d t = overlapCorner(
            srcplanes[ids[0]],
            srcplanes[ids[1]],
            srcplanes[ids[2]],
            tgtplanes[planecorrespondences[ids[0]]],
            tgtplanes[planecorrespondences[ids[1]]],
            tgtplanes[planecorrespondences[ids[2]]]);
    return AlignmentResult(t, 0);
}

AlignmentResult alignICP(
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
    return AlignmentResult(icp.getFinalTransformation().cast<double>(), icp.getFitnessScore());
}


AlignmentResult align(
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
    return AlignmentResult(Matrix4d::Identity(), numeric_limits<double>::infinity());
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
