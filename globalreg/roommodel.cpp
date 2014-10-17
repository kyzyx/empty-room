#include "roommodel.h"
#include "util.h"
#include <limits>
#include <pcl/common/transforms.h>

using namespace std;
using namespace Eigen;
using namespace pcl;

const double ANGLETHRESHOLD = M_PI/8;
const double WALLMERGETHRESHOLD = 0.24;
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
    rotationaxes.push_back(-1);
    rotationangles.push_back(0);
    if (cumxforms.size()) {
        cumxforms.push_back(cumxforms.back()*transform);
    } else {
        cumxforms.push_back(transform);
    }
    weights.push_back(rmse);
    for (int i = 0; i < 3; ++i) {
        constrained[i].push_back(-1);
        seen[i].push_back(false);
    }

    vector<Vector4d> newplanes;
    vector<Vector4d> origplanes;
    int nconstrained = 0;
    for (int i = 0; i < planes.size(); ++i) {
        Vector4d p = transformPlane(planes[i], cumxforms.back());
        for (int j = 0; j < 3; ++j) {
            if (abs(p.head(3).dot(axes[j])) > cos(ANGLETHRESHOLD)) {
                seen[j].back() = true;
                if (alignedto[i]) {
                    constrained[j].back() = nconstrained++;
                    if (constrained[j].size() == 1 || (constrained[j].size() > 1 && constrained[j][constrained[j].size()-2] == -1)) {
                        newplanes.push_back(p);
                        origplanes.push_back(planes[i]);
                    }
                }
                break;
            }
        }
    }
    allplanes.push_back(vector<Vector4d>(origplanes));

    if (newplanes.size()) {
        for (int i = 0; i < newplanes.size(); ++i) {
            if (clouds.size() > 1) {
                int frame = clouds.size()-2;
                distributeRotation(newplanes[i], frame);
                break;
            }
        }
    }
    printf("Ntracked: %d %d %d\n", constrained[0].back(), constrained[1].back(), constrained[2].back());
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
                    printf("%d New wall (oriented %d) at %.3f\n", frame, k, plane(3));
                } else {
                    printf("%d Wall (oriented %d) matched at %.3f\n", frame, k, plane(3));
                    distributeTranslation(k, best-plane(3), frame, lastframe);
                    distributeTranslation(k, -(best-plane(3)), lastframe, frame);
                    ub->second = frame;
                }
            } else {
                roomplanes[k].insert(make_pair(plane(3),frame));
                printf("%d New wall (oriented %d) at %.3f\n", frame, k, plane(3));
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
            expected = j;
        } else if (constrained[j][frame] > -1) {
            if (tracked == -1 || constrained[tracked][frame] > constrained[j][frame])
                tracked = j;
        }
    }
    printf("New plane: %.3f %.3f %.3f %.3f\n", plane(0), plane(1), plane(2), plane(3));
    int k = frame-1;
    while (tracked == -1 && k >= 0) {
        for (int j = 0; j < 3; ++j) {
            if (j == expected) continue;
            if (constrained[j][k] > -1) {
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
    for (i = start; i <= frame; ++i) {
        rotationangles[i] = angle*weights[i]/total;
        rotationaxes[i] = tracked;
        cout << i << ": " << weights[i]/total << endl;
    }
    recomputeCumulativeTransforms(start);
}

void RoomModel::distributeTranslation(
        int orientation, double translation,
        int endframe, int startframe, double residualweight)
{
    int n = adjustments.size();
    if (endframe < startframe) endframe += n;
    int o2 = orientation/2;
    double total = residualweight;
    for (int i = startframe+1; i <= endframe; ++i) {
        if (constrained[o2][i%n] == -1) total += weights[i%n];
    }
    printf("Translation %.3f; Closing loop to frame %d\n", translation, startframe);
    for (int i = startframe+1; i <= endframe; ++i) {
        if (constrained[o2][i%n] == -1) {
            Matrix4d t = Matrix4d::Identity();
            Vector3d transl = ((orientation&1)?1:-1)*axes[o2]*translation*weights[i%n]/total;
            t.topRightCorner(3,1) = transl;
            cout << i%n << ": " << weights[i%n]/total << endl;
            adjustments[i%n] = t*adjustments[i%n];
        }
    }
    recomputeCumulativeTransforms(endframe>n?0:startframe);
}

void RoomModel::recomputeCumulativeTransforms(int start) {
    if (start == 0) {
        cumxforms[0] = adjustments[0]*xforms[0];
        start++;
    }
    PointCloud<PointXYZ>::Ptr tcloud1(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr tcloud2(new PointCloud<PointXYZ>);
    for (int i = start; i < xforms.size(); ++i) {
        Matrix4d t = Matrix4d::Identity();
        if (rotationaxes[i] > -1) {
            Matrix4d tmpxform = cumxforms[i-1]*xforms[i];
            transformPointCloud(*clouds[i], *tcloud2, tmpxform);
            transformPointCloud(*clouds[i-1], *tcloud1, cumxforms[i-1]);
            Vector3d avgpt = cloudMidpoint(tcloud1, tcloud2);
            t.topRightCorner(3,1) = -avgpt;
            Matrix4d r = Matrix4d::Identity();
            r.topLeftCorner(3,3) = AngleAxisd(rotationangles[i], axes[rotationaxes[i]]).matrix();
            t = t.inverse()*r*t;
        }

        cumxforms[i] = adjustments[i]*t*cumxforms[i-1]*xforms[i];
    }
}

Matrix4d RoomModel::getTransform(int n) {
    if (n == 0) return xforms[n];
    return cumxforms[n-1].inverse()*cumxforms[n];
}

void RoomModel::closeLoop(Matrix4d transform, double rmse) {
    Vector3d t = transform.topRightCorner(3,1);
    distributeTranslation(1, axes[0].dot(t), clouds.size()-1, 0, 0);
    distributeTranslation(3, axes[1].dot(t), clouds.size()-1, 0, 0);
    distributeTranslation(5, axes[2].dot(t), clouds.size()-1, 0, 0);
}

void RoomModel::makeWallsConsistent() {
    for (int z = 0; z < clouds.size(); ++z) {
        for (int i = 0; i < allplanes[z].size(); ++i) {
            Vector4d p = transformPlane(allplanes[z][i], cumxforms[z]);
            checkLoopClosure(p, z?z-1:z);
        }
    }
}
