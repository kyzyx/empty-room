#include "pairwise.h"
#include "findplanes.h"
#include "util.h"
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>

using namespace pcl;
using namespace Eigen;
using namespace std;

const double ANGLETHRESHOLD = M_PI/9;
const double REDOTHRESHOLD = 0.03;
const int MAXITERATIONS = 50;          // ICP iterations for edge and plane

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
    double angle = safe_acos(vs2.dot(vt2));
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

AlignmentResult alignICP(
        PointCloud<PointXYZ>::ConstPtr src,
        PointCloud<PointXYZ>::ConstPtr tgt,
        vector<Vector4d>& srcplanes, vector<int>& srcids,
        vector<Vector4d>& tgtplanes, vector<int>& tgtids,
        vector<int>& planecorrespondences,
        int maxiterations)
{
    PointCloud<PointXYZ>::Ptr s(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr t(new PointCloud<PointXYZ>);
    vector<int> tmp;
    removeNaNFromPointCloud(*src, *s, tmp);
    removeNaNFromPointCloud(*tgt, *t, tmp);

    IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    icp.setInputSource(s);
    icp.setInputTarget(t);
    icp.setMaximumIterations(maxiterations);
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
            return alignICP(src, tgt, srcplanes, srcids, tgtplanes, tgtids, planecorrespondences, 100);
            break;
        case 1:
            cout << "Aligning plane to plane" << endl;
            {
                AlignmentResult c = alignPlaneToPlane(src, tgt, srcplanes, srcids, tgtplanes, tgtids, planecorrespondences, MAXITERATIONS);
                cout << "Final RMSE: " << c.error << endl;
                return c;
            }
            break;
        case 2:
            cout << "Aligning edge to edge" << endl;
            {
                AlignmentResult c = alignEdgeToEdge(src, tgt, srcplanes, srcids, tgtplanes, tgtids, planecorrespondences, MAXITERATIONS);
                if (c.error > REDOTHRESHOLD) {
                    cout << "Redoing with plane to plane" << endl;
                    AlignmentResult c2 = alignPlaneToPlane(src, tgt, srcplanes, srcids, tgtplanes, tgtids, planecorrespondences, MAXITERATIONS);
                    if (c2.error < c.error) {
                        cout << "Final RMSE: " << c2.error << endl;
                        return c2;
                    }
                }
                cout << "Final RMSE: " << c.error << endl;
                return c;
            }
            break;
        case 3:
            cout << "Aligning corner to corner" << endl;
            {
                AlignmentResult c = alignCornerToCorner(src, tgt, srcplanes, srcids, tgtplanes, tgtids, planecorrespondences);
                if (c.error > REDOTHRESHOLD) {
                    vector<int> ids;
                    for (int i = 0; i < planecorrespondences.size(); ++i) {
                        if (planecorrespondences[i] > -1) ids.push_back(i);
                    }
                    for (int i = 0; i < 3; ++i) {
                        cout << "Redoing with edge to edge " << i+1 << endl;
                        vector<int> pc2(planecorrespondences);
                        pc2[ids[i]] = -1;
                        AlignmentResult c2 = alignEdgeToEdge(src, tgt, srcplanes, srcids, tgtplanes, tgtids, pc2, MAXITERATIONS);
                        if (c2.error < c.error) {
                            c = c2;
                        }
                    }
                }
                cout << "Final RMSE: " << c.error << endl;
                return c;
            }
            break;
        default:
            cerr << "Error! " << numcorrespondences << " correspondences found!" << endl;
    }
    return AlignmentResult(Matrix4d::Identity(), numeric_limits<double>::infinity());
}
