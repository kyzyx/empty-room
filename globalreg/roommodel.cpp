#include "roommodel.h"
#include "util.h"
#include <limits>
#include <pcl/common/transforms.h>

using namespace std;
using namespace Eigen;
using namespace pcl;

const double ANGLETHRESHOLD = M_PI/8;
const double WALLMERGETHRESHOLD = 0.05;
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

    vector<Vector4d> currplanes;
    for (int i = 0; i < planes.size(); ++i) {
        Vector4d p = transformPlane(planes[i], cumxforms.back());
        for (int j = 0; j < 3; ++j) {
            if (abs(p.head(3).dot(axes[j])) > cos(ANGLETHRESHOLD)) {
                currplanes.push_back(p);
                seen[j].back() = true;
                if (alignedto[i]) constrained[j].back() = true;
                break;
            }
        }
    }
    vector<Vector4d> newplanes;
    for (int i = 0; i < currplanes.size(); ++i) {
        bool matched = false;
        for (int j = 0; j < prevplanes.size(); ++j) {
            if (currplanes[i].head(3).dot(prevplanes[j].head(3)) > cos(ANGLETHRESHOLD) &&
                abs(currplanes[i](3)-prevplanes[j](3)) < WALLMERGETHRESHOLD) {
                matched = true;
                break;
            }
        }
        if (!matched) {
            newplanes.push_back(currplanes[i]);
        }
    }

    if (newplanes.size()) {
        for (int i = 0; i < newplanes.size(); ++i) {
            if (clouds.size() > 1) {
                distributeRotation(newplanes[i]);
            }
        }
    }
    printf("Ntracked: %d %d %d\n", constrained[0].back()?1:0, constrained[1].back()?1:0, constrained[2].back()?1:0);
    prevplanes = currplanes;
    // Check for new walls
    // checkLoopClosure(newplanes);
}

void RoomModel::checkLoopClosure(vector<Vector4d>& planes) {
    for (int i = 0; i < planes.size(); ++i) {
        Vector4d p = planes[i];
        for (int j = 0; j < 3; ++j) {
            if (abs(p.head(3).dot(axes[j])) > cos(ANGLETHRESHOLD)) {
                int k = p.head(3).dot(axes[j])>0?2*j:2*j+1;
                if (roomplanes[k].size()) {
                    double best = numeric_limits<double>::infinity();
                    map<double, int>::iterator ub = roomplanes[k].upper_bound(p(3));
                    if (ub != roomplanes[k].end()) {
                        best = ub->first;
                    }
                    if (ub != roomplanes[k].begin()) {
                        --ub;
                        if (abs(best-p(3)) > abs(ub->first - p(3))) {
                            best = ub->first;
                        }
                    }

                    if (abs(best-p(3)) > WALLMERGETHRESHOLD) {
                        roomplanes[k].insert(make_pair(p(3),clouds.size()-1));
                    } else {
                        // FIXME: Close loop!
                    }
                } else {
                    roomplanes[k].insert(make_pair(p(3),clouds.size()-1));
                }
            }
        }
    }
}

void RoomModel::distributeRotation(Vector4d plane) {
    int expected = -1;
    int tracked = -1;
    int other = -1;
    for (int j = 0; j < 3; ++j) {
        if (abs(plane.head(3).dot(axes[j])) > cos(ANGLETHRESHOLD)) {
            if (constrained[j][weights.size()-2]) {
                return; // Just a plane we stopped tracking, not a new orientation
            }
            expected = j;
        } else if (constrained[j].back()) {
            if (tracked == -1) tracked = j;
            else {
                return; // Already aligned
            }
        }
    }
    printf("New plane: %.3f %.3f %.3f %.3f\n", plane(0), plane(1), plane(2), plane(3));
    other = 3 - expected - tracked;

    Vector3d expectedNormal = axes[expected];
    double angle = plane.head(3).dot(axes[expected]);
    if (angle < 0) {
        expectedNormal = -expectedNormal;
        angle = -angle;
    }
    angle = safe_acos(angle);
    if (((Vector3d) plane.head(3)).cross(expectedNormal).dot(axes[tracked]) < 0) angle = -angle;

    double total = weights[weights.size()-1];
    int i = constrained[0].size()-2;
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
    for (i = start; i < constrained[0].size(); ++i) {
        transformPointCloud(*clouds[i], *tcloud2, cumxforms[i]);
        transformPointCloud(*clouds[i-1], *tcloud1, cumxforms[i-1]);
        Vector3d avgpt = cloudMidpoint(tcloud1, tcloud2);
        Matrix4d t = Matrix4d::Identity();
        t.topRightCorner(3,1) = -avgpt;
        Matrix4d r = Matrix4d::Identity();
        r.topLeftCorner(3,3) = AngleAxisd(angle*weights[i]/total, axes[tracked]).matrix();
        cout << i << ": " << weights[i]/total << endl;
        adjustments[i] = t.inverse()*r*t;
    }
    recomputeCumulativeTransforms(start);
}

void RoomModel::distributeTranslation(Vector4d plane, double matched) {

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
