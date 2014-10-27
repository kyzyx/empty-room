#include "pairwise.h"
#include "findplanes.h"
#include "util.h"

using namespace pcl;
using namespace Eigen;
using namespace std;

const double ANGLETHRESHOLD = M_PI/12;
const double MATCHANGLETHRESHOLD = M_PI/6;
const double MAXTRANSLATION = 0.25;
const double OFFSETTHRESHOLD = 0.06;

inline bool isParallel(Vector4d a, Vector4d b) {
    return abs(a.head(3).dot(b.head(3))) > cos(ANGLETHRESHOLD);
}
inline bool isParallel(Vector3d a, Vector3d b) {
    return abs(a.head(3).dot(b.head(3))) > cos(ANGLETHRESHOLD);
}


void recomputePlanesManhattan(
        Vector3d p1, Vector3d p2,
        PointCloud<PointXYZ>::ConstPtr cloud,
        vector<Vector4d>& planes,
        vector<int>& ids)
{
    vector<Vector3d> axes;
    axes.push_back(p1);
    axes.push_back(p2);
    axes.push_back(p1.cross(p2));
    // Take largest planes compatible with coordinate system
    vector<Vector3d> compatible;
    for (int i = 0; i < planes.size(); ++i) {
        for (int j = 0; j < axes.size(); ++j) {
            if (abs(planes[i].head(3).dot(axes[j])) > cos(MATCHANGLETHRESHOLD)) {
                compatible.push_back(planes[i].head(3));
                swap(axes[j], axes[axes.size()-1]);
                axes.resize(axes.size()-1);
                break;
            }
        }
    }
    // Refine coordinate system
    Vector3d q1 = compatible[0];
    Vector3d qp = compatible[1];
    Vector3d q2 = q1.cross(qp);
    q2.normalize();

    axes.clear();
    axes.push_back(q1);
    axes.push_back(q2);
    axes.push_back(q1.cross(q2).normalized());
    vector<bool> consistent(planes.size(), false);
    vector<int> relabel;
    vector<Vector4d> origplanes(planes);
    planes.clear();
    int n = 0;
    for (int i = 0; i < origplanes.size(); ++i) {
        bool consistent = false;
        for (int j = 0; j < 3; ++j) {
            if (isParallel(origplanes[i].head(3), axes[j])) {
                consistent = true;
                if (origplanes[i].head(3).dot(axes[j]) > 0) {
                    origplanes[i].head(3) = axes[j];
                } else {
                    origplanes[i].head(3) = -axes[j];
                }
                planes.push_back(origplanes[i]);
                relabel.push_back(n++);
                break;
            }
        }
        if (!consistent) {
            relabel.push_back(-1);
        }
    }

    vector<double> offsets(planes.size(), 0);
    vector<int> counts(planes.size(), 0);
    for (int i = 0; i < cloud->size(); ++i) {
        if (ids[i] >= 0) ids[i] = relabel[ids[i]];
        if (ids[i] >= 0) {
            counts[ids[i]]++;
            offsets[ids[i]] -= Vector3d(cloud->at(i).x, cloud->at(i).y, cloud->at(i).z).dot(planes[ids[i]].head(3));
        }
    }
    for (int i = 0; i < planes.size(); ++i) {
        planes[i](3) = offsets[i]/counts[i];
    }
}

void findParallelCorrespondences(
        vector<pair<double,int> >& splanes, vector<pair<double,int> >& tplanes,
        vector<Vector4d>& srcplanes, vector<Vector4d>& tgtplanes,
        vector<int>& planecorrespondences,
        bool multipleAllowed)
{
    if (splanes.empty() || tplanes.empty()) return;
    if (splanes.size() == 1) {
        double best = 0.5;
        for (int i = 0; i < tplanes.size(); ++i) {
            if (srcplanes[splanes[0].second].head(3).dot(tgtplanes[tplanes[i].second].head(3)) > 0) {
                if (abs(tplanes[i].first - splanes[0].first) < best) {
                    best = abs(tplanes[i].first - splanes[0].first);
                    planecorrespondences[splanes[0].second] = tplanes[i].second;
                }
            }
        }
    } else if (tplanes.size() == 1) {
        double best = 0.5;
        for (int i = 0; i < splanes.size(); ++i) {
            if (srcplanes[splanes[i].second].head(3).dot(tgtplanes[tplanes[0].second].head(3)) > 0) {
                if (abs(tplanes[0].first - splanes[i].first) < best) {
                    best = abs(tplanes[0].first - splanes[i].first);
                    planecorrespondences[splanes[i].second] = tplanes[0].second;
                }
            }
        }
    } else {
        sort(splanes.begin(), splanes.end());
        sort(tplanes.begin(), tplanes.end());
        vector<double> offsets;
        for (int i = 0; i < splanes.size(); ++i) {
            for (int j = 0; j < tplanes.size(); ++j) {
                if (srcplanes[splanes[i].second].head(3).dot(tgtplanes[tplanes[j].second].head(3)) > 0) {
                    offsets.push_back(splanes[i].first - tplanes[j].first);
                }
            }
        }
        sort(offsets.begin(), offsets.end());
        int bestmatches = 0;
        double bestoffset = 0;
        for (int i = 0; i < offsets.size(); ++i) {
            int matches = 1;
            double avg = offsets[i];
            for (int j = i+1; j < offsets.size() && offsets[j] - offsets[i] < OFFSETTHRESHOLD; ++j) {
                ++matches;
                avg += offsets[j];
            }
            if (matches > bestmatches) {
                bestmatches = matches;
                bestoffset = avg/matches;
            } else if (matches == bestmatches) {
                if (abs(bestoffset) > abs(avg/matches)) bestoffset = avg/matches;
            }
        }
        bool done = false;
        int first = planecorrespondences.size();
        const int MAGIC = 10;
        for (int i = 0; i < splanes.size(); ++i) {
            for (int j = 0; j < tplanes.size(); ++j) {
                if (srcplanes[splanes[i].second].head(3).dot(tgtplanes[tplanes[j].second].head(3)) > 0) {
                    if (abs((splanes[i].first - tplanes[j].first) - bestoffset) < OFFSETTHRESHOLD/2) {
                        if (multipleAllowed) planecorrespondences[splanes[i].second] = tplanes[j].second;
                        else planecorrespondences[splanes[i].second] = -MAGIC - tplanes[j].second;
                        if (splanes[i].second < first) first = splanes[i].second;
                        done = true;
                        break;
                    }
                }
            }
        }
        if (!multipleAllowed) planecorrespondences[first] = -planecorrespondences[first] - MAGIC;
    }
}


int findCandidateCorrespondences(
        vector<Vector4d>& srcplanes,
        vector<Vector4d>& tgtplanes,
        vector<int>& planecorrespondences)
{
    vector<vector<pair<double,int> > > splanes;
    vector<vector<pair<double,int> > > tplanes;
    vector<Vector3d> sdirs;
    vector<Vector3d> tdirs;

    // Find all src plane directions
    vector<bool> assigned(srcplanes.size(), false);
    for (int i = 0; i < srcplanes.size(); ++i) {
        if (assigned[i]) continue;
        sdirs.push_back(srcplanes[i].head(3));
        splanes.push_back(vector<pair<double,int> >());

        splanes.back().push_back(make_pair(srcplanes[i](3), i));
        assigned[i] = true;
        for (int j = i+1; j < srcplanes.size(); ++j) {
            if (isParallel(srcplanes[i], srcplanes[j])) {
                splanes.back().push_back(make_pair(srcplanes[j](3), j));
                assigned[j] = true;
            }
        }
    }

    // Find all tgt plane directions
    assigned.clear();
    assigned.resize(tgtplanes.size(), false);
    for (int i = 0; i < tgtplanes.size(); ++i) {
        if (assigned[i]) continue;
        tdirs.push_back(tgtplanes[i].head(3));
        tplanes.push_back(vector<pair<double,int> >());

        tplanes.back().push_back(make_pair(tgtplanes[i](3), i));
        assigned[i] = true;
        for (int j = i+1; j < tgtplanes.size(); ++j) {
            if (isParallel(tgtplanes[i], tgtplanes[j])) {
                tplanes.back().push_back(make_pair(tgtplanes[j](3), j));
                assigned[j] = true;
            }
        }
    }

    planecorrespondences.clear();
    planecorrespondences.resize(srcplanes.size(), -1);
    for (int i = 0; i < sdirs.size(); ++i) {
        int bestj = -1;
        double besta = 0;
        for (int j = 0; j < tdirs.size(); ++j) {
            double cosa = abs(sdirs[i].dot(tdirs[j]));
            if (cosa > cos(MATCHANGLETHRESHOLD) && cosa > besta) {
                besta = cosa;
                bestj = j;
            }
        }
        if (bestj == -1) continue;
        findParallelCorrespondences(splanes[i], tplanes[bestj], srcplanes, tgtplanes, planecorrespondences, false);
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

    int numcorrespondences = 0;
    for (int i = 0; i < planecorrespondences.size(); ++i) {
        if (planecorrespondences[i] > -1) ++numcorrespondences;
    }
    return numcorrespondences;
}

void selectManhattanSystem(
        vector<Vector4d>& srcplanes,
        vector<Vector4d>& tgtplanes,
        vector<int>& planecorrespondences)
{
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
}


int findManhattanCorrespondences(Vector3d axis1, Vector3d axis2,
        vector<Vector4d>& srcplanes, vector<Vector4d>& tgtplanes,
        vector<int>& planecorrespondences)
{
    // Assume planes are already aligned
    // For each dominant direction, find the optimal offset
    vector<pair<double,int> > splanes[3];
    vector<pair<double,int> > tplanes[3];
    vector<Vector3d> axes;
    axes.push_back(axis1);
    axes.push_back(axis2);
    axes.push_back(axis1.cross(axis2));
    for (int i = 0; i < srcplanes.size(); ++i) {
        for (int j = 0; j < 3; ++j) {
            if (isParallel(srcplanes[i].head(3), axes[j])) {
                splanes[j].push_back(make_pair(srcplanes[i](3), i));
                break;
            }
        }
    }
    for (int i = 0; i < tgtplanes.size(); ++i) {
        for (int j = 0; j < 3; ++j) {
            if (abs(tgtplanes[i].head(3).dot(axes[j])) > cos(MATCHANGLETHRESHOLD)) {
                tplanes[j].push_back(make_pair(tgtplanes[i](3),i));
                break;
            }
        }
    }
    planecorrespondences.clear();
    planecorrespondences.resize(srcplanes.size(), -1);
    for (int i = 0; i < 3; ++i) {
        findParallelCorrespondences(splanes[i], tplanes[i], srcplanes, tgtplanes, planecorrespondences, false);
    }

    int numcorrespondences = 0;
    for (int i = 0; i < planecorrespondences.size(); ++i) {
        if (planecorrespondences[i] > -1) ++numcorrespondences;
    }
    return numcorrespondences;
}

void reduceCorrespondences(vector<Vector4d>& planes, vector<int>& planecorrespondences)
{
    for (int i = 0; i < planecorrespondences.size(); ++i) {
        if (planecorrespondences[i] > -1) {
            int best = i;
            int orig = planecorrespondences[i];
            double rearmost = planes[i](3);
            planecorrespondences[i] = -1;
            for (int j = i+1; j < planecorrespondences.size(); ++j) {
                if (planecorrespondences[j] > -1 && isParallel(planes[i], planes[j])) {
                    if (planes[j](3) > rearmost) {
                        rearmost = planes[j](3);
                        best = j;
                        orig = planecorrespondences[j];
                    }
                    planecorrespondences[j] = -1;
                }
            }
            planecorrespondences[best] = orig;
        }
    }
}

int findPlaneCorrespondences(
        PointCloud<PointXYZ>::ConstPtr src,
        PointCloud<PointXYZ>::ConstPtr tgt,
        vector<Vector4d>& srcplanes, vector<int>& srcids,
        vector<Vector4d>& tgtplanes, vector<int>& tgtids,
        vector<int>& planecorrespondences)
{
    return findPlaneCorrespondencesFiltered(src, tgt, srcplanes, srcids, tgtplanes, tgtids, planecorrespondences, false);
}

int findPlaneCorrespondencesFiltered(
        PointCloud<PointXYZ>::ConstPtr src,
        PointCloud<PointXYZ>::ConstPtr tgt,
        vector<Vector4d>& srcplanes, vector<int>& srcids,
        vector<Vector4d>& tgtplanes, vector<int>& tgtids,
        vector<int>& planecorrespondences,
        bool prefiltered)
{
    planecorrespondences.resize(srcplanes.size(), -1);
    // Make plane offsets relative to center of frame
    Vector3d avgpt = cloudMidpoint(src, tgt);
    for (int i = 0; i < srcplanes.size(); ++i) {
        srcplanes[i](3) += avgpt.dot(srcplanes[i].head(3));
    }
    for (int i = 0; i < tgtplanes.size(); ++i) {
        tgtplanes[i](3) += avgpt.dot(tgtplanes[i].head(3));
    }
    // Generate possible correspondences
    int numcorrespondences = findCandidateCorrespondences(srcplanes, tgtplanes, planecorrespondences);
    //if (!prefiltered) {
        // Isolate correspondences within a single Manhattan coordinate system
        selectManhattanSystem(srcplanes, tgtplanes, planecorrespondences);
    //}
    // Isolate at most one plane in each direction to match
    reduceCorrespondences(srcplanes, planecorrespondences);
    // Recenter for analysis
    for (int i = 0; i < srcplanes.size(); ++i) {
        srcplanes[i](3) -= avgpt.dot(srcplanes[i].head(3));
    }
    for (int i = 0; i < tgtplanes.size(); ++i) {
        tgtplanes[i](3) -= avgpt.dot(tgtplanes[i].head(3));
    }

    // Recount correspondences
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
        Vector3d p1, p2, pp;
        if (!prefiltered) {
            // Create tentative coordinate system
            p1 = tgtplanes[planecorrespondences[planeids[0]]].head(3);
            pp = tgtplanes[planecorrespondences[planeids[1]]].head(3);
            p2 = p1.cross(pp);
            p2.normalize();
            recomputePlanesManhattan(p1, p2, tgt, tgtplanes, tgtids);
        }
        p1 = srcplanes[planeids[0]].head(3);
        pp = srcplanes[planeids[1]].head(3);
        p2 = p1.cross(pp);
        p2.normalize();
        recomputePlanesManhattan(p1, p2, src, srcplanes, srcids);

        // Make plane offsets relative to center of frame
        for (int i = 0; i < srcplanes.size(); ++i) {
            srcplanes[i](3) += avgpt.dot(srcplanes[i].head(3));
        }
        for (int i = 0; i < tgtplanes.size(); ++i) {
            tgtplanes[i](3) += avgpt.dot(tgtplanes[i].head(3));
        }

        // Recompute correspondences with new planes
        planecorrespondences.clear();
        planecorrespondences.resize(srcplanes.size(), -1);
        numcorrespondences = findManhattanCorrespondences(p1, p2, srcplanes, tgtplanes, planecorrespondences);
        reduceCorrespondences(srcplanes, planecorrespondences);

        // Recenter for return
        for (int i = 0; i < srcplanes.size(); ++i) {
            srcplanes[i](3) -= avgpt.dot(srcplanes[i].head(3));
        }
        for (int i = 0; i < tgtplanes.size(); ++i) {
            tgtplanes[i](3) -= avgpt.dot(tgtplanes[i].head(3));
        }

        // Final count of correspondences
        numcorrespondences = 0;
        for (int i = 0; i < planecorrespondences.size(); ++i) {
            if (planecorrespondences[i] > -1) {
                ++numcorrespondences;
            }
        }
    }

    return numcorrespondences;
}
