#include "roommodel.h"
#include "util.h"
#include <limits>
#include <pcl/common/transforms.h>

using namespace std;
using namespace Eigen;
using namespace pcl;

const double ANGLETHRESHOLD = M_PI/8;
const double WALLMERGETHRESHOLD = 0.085;
const double EQUALTHRESHOLD = M_PI/16;

void RoomModel::setAxes(Vector3d a, Vector3d b) {
    axes.clear();
    axes.push_back(a);
    axes.push_back(b);
    axes.push_back(a.cross(b));
}

void RoomModel::addCloud(
        PointCloud<PointXYZ>::ConstPtr cloud,
        vector<Vector4d>& planes,
        vector<bool>& alignedto,
        Matrix4d transform,
        double rmse)
{
    clouds.push_back(cloud);
    xforms.push_back(transform);
    adjustments.push_back(Matrix4d::Identity());
    if (cumxforms.size()) {
        cumxforms.push_back(cumxforms.back()*transform);
    } else {
        cumxforms.push_back(transform);
    }
    weights.push_back(rmse);
    for (int i = 0; i < 3; ++i) {
        constrained[i].push_back(false);
        seen[i].push_back(false);
    }

    vector<Vector4d> newplanes;
    for (int i = 0; i < planes.size(); ++i) {
        Vector4d p = transformPlane(planes[i], cumxforms.back());
        for (int j = 0; j < 3; ++j) {
            if (abs(p.head(3).dot(axes[j])) > cos(ANGLETHRESHOLD)) {
                seen[j].back() = true;
                if (alignedto[i]) {
                    constrained[j].back() = true;
                    if (constrained[j].size() > 1 && !constrained[j][constrained[j].size()-2]) newplanes.push_back(p);
                    else if (constrained[j].size() == 1) checkLoopClosure(p, 1);
                }
                break;
            }
        }
    }

    if (newplanes.size()) {
        for (int i = 0; i < newplanes.size(); ++i) {
            if (clouds.size() > 1) {
                int frame = clouds.size()-2;
                distributeRotation(newplanes[i], frame);
                checkLoopClosure(newplanes[i], frame);
            }
        }
    }
    printf("Ntracked: %d %d %d\n", constrained[0].back()?1:0, constrained[1].back()?1:0, constrained[2].back()?1:0);
}

void RoomModel::checkLoopClosure(Vector4d plane, int frame) {
    for (int i = 0; i < 3; ++i) {
        if (abs(plane.head(3).dot(axes[i])) > cos(ANGLETHRESHOLD)) {
            int k = plane.head(3).dot(axes[i])>0?2*i:2*i+1;
            if (roomplanes[k].size()) {
                double best = numeric_limits<double>::infinity();
                int lastframe = -1;
                map<double, int>::iterator ub = roomplanes[k].upper_bound(plane(3));
                if (ub != roomplanes[k].end()) {
                    best = ub->first;
                    lastframe = ub->second;
                }
                if (ub != roomplanes[k].begin()) {
                    --ub;
                    if (abs(best-plane(3)) > abs(ub->first - plane(3))) {
                        best = ub->first;
                        lastframe = ub->second;
                    } else {
                        ++ub;
                    }
                }

                if (abs(best-plane(3)) > WALLMERGETHRESHOLD) {
                    roomplanes[k].insert(make_pair(plane(3),frame));
                    printf("New wall (oriented %d) at %.3f\n", k, plane(3));
                } else {
                    printf("Wall (oriented %d) matched at %.3f\n", k, plane(3));
                    distributeTranslation(k, best-plane(3), frame, lastframe);
                    ub->second = frame;
                }
            } else {
                roomplanes[k].insert(make_pair(plane(3),frame));
                printf("New wall (oriented %d) at %.3f\n", k, plane(3));
            }
            break;
        }
    }
}

void RoomModel::distributeRotation(Vector4d plane, int frame) {
    int expected = -1;
    int tracked = -1;
    int other = -1;
    for (int j = 0; j < 3; ++j) {
        if (abs(plane.head(3).dot(axes[j])) > cos(ANGLETHRESHOLD)) {
            if (constrained[j][frame-1]) {
                return; // Just a plane we stopped tracking, not a new orientation
            }
            expected = j;
        } else if (constrained[j][frame]) {
            if (tracked == -1) tracked = j;
            else {
                return; // Already aligned
            }
        }
    }
    printf("New plane: %.3f %.3f %.3f %.3f\n", plane(0), plane(1), plane(2), plane(3));
    int k = frame-1;
    while (tracked == -1 && k >= 0) {
        for (int j = 0; j < 3; ++j) {
            if (j == expected) continue;
            if (constrained[j][k]) {
                tracked = j;
            }
        }
        --k;
    }
    if (k == -1) return;
    other = 3 - expected - tracked;

    Vector3d expectedNormal = axes[expected];
    double angle = plane.head(3).dot(axes[expected]);
    if (angle < 0) {
        expectedNormal = -expectedNormal;
        angle = -angle;
    }
    angle = safe_acos(angle);
    if (((Vector3d) plane.head(3)).cross(expectedNormal).dot(axes[tracked]) < 0) angle = -angle;

    double total = weights[frame];
    int i = frame-1;
    for (; i >= 0; --i) {
        if (seen[expected][i] || seen[other][i]) {
            break;
        }
        total += weights[i];
    }
    printf("Rotation Angle %.3f; Closing loop to frame %d\n", angle, i);
    int start = i+1;
    PointCloud<PointXYZ>::Ptr tcloud1(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr tcloud2(new PointCloud<PointXYZ>);
    for (i = start; i <= frame; ++i) {
        transformPointCloud(*clouds[i], *tcloud2, cumxforms[i]);
        transformPointCloud(*clouds[i-1], *tcloud1, cumxforms[i-1]);
        Vector3d avgpt = cloudMidpoint(tcloud1, tcloud2);
        Matrix4d t = Matrix4d::Identity();
        t.topRightCorner(3,1) = -avgpt;
        Matrix4d r = Matrix4d::Identity();
        r.topLeftCorner(3,3) = AngleAxisd(angle*weights[i]/total, axes[tracked]).matrix();
        cout << i << ": " << weights[i]/total << endl;
        adjustments[i] = t.inverse()*r*t*adjustments[i];
    }
    recomputeCumulativeTransforms(start);
}

void RoomModel::distributeTranslation(
        int orientation, double translation,
        int endframe, int startframe)
{
    int o2 = orientation/2;
    double total = 0;
    for (int i = startframe+1; i <= endframe; ++i) {
        if (!constrained[o2][i]) total += weights[i];
    }
    printf("Translation %.3f; Closing loop to frame %d\n", translation, startframe);
    for (int i = startframe+1; i <= endframe; ++i) {
        if (!constrained[o2][i]) {
            Matrix4d t = Matrix4d::Identity();
            Vector3d transl = Vector3d::Zero();
            transl = ((orientation&1)?1:-1)*axes[o2]*translation*weights[i]/total;
            t.topRightCorner(3,1) = transl;
            cout << i << ": " << weights[i]/total << endl;
            adjustments[i] = t*adjustments[i];
        }
    }
    recomputeCumulativeTransforms(startframe);
}

void RoomModel::recomputeCumulativeTransforms(int start) {
    if (start == 0) {
        cumxforms[0] = adjustments[0]*xforms[0];
        start++;
    }
    for (int i = start; i < xforms.size(); ++i) {
        cumxforms[i] = adjustments[i]*cumxforms[i-1]*xforms[i];
    }
}
