#include "pairwise.h"
#include "plane2plane.h"
#include "util.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>

using namespace pcl;
using namespace Eigen;
using namespace std;

const double ANGLETHRESHOLD = M_PI/8;         // Angle between normals to be considered parallel
const double MAXCORRESPONDENCEDISTANCE = 0.2; // Hard cutoff of correspondence distance
const int EDGEREMOVAL = 60;                   // Remove this many pixels on the boundaries of depth images
const int DISCONTINUITYBORDERWIDTH = 4;       // Keep this many pixels on plane edges
const double DISCONTINUITYTHRESHOLD = 0.1;    // Discontinuity if distance > threshold
const double SEARCHWIDTH = 0.01;       // Data structure discretization
const double errthreshold = 0.00005;   // Stop ICP after error below this threshold
const double MINTRANSLATION = 0.0001;  // Stop ICP after translation below this threshold
const double MINROTATION = M_PI/40;  // Stop ICP after rotation angle below this threshold

void filterLabelled(PointCloud<PointXYZ>::Ptr cloud, vector<int>& labels, int label, bool negative=true)
{
    PointIndices::Ptr inds(new PointIndices());
    for (int i = 0; i < labels.size(); ++i) {
        if (labels[i] == label) inds->indices.push_back(i);
    }
    ExtractIndices<PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inds);
    extract.setNegative(negative);
    extract.filter(*cloud);
    vector<int> tmp;
    removeNaNFromPointCloud(*cloud, *cloud, tmp);
}

PointXYZ LayeredKdTrees::nearest(PointXYZ p, double radius) const
{
    if (!isValid(p)) return NaNPt;
    const std::vector<Tree*>& trees = p.z>0?postrees:negtrees;
    int ind = abs(p.z/thickness);
    if (ind >= trees.size()) return NaNPt;
    pcl::PointXYZ p1 = nearestInTree(trees, ind, p, radius);
    if (abs(p.z/thickness) - ind < 0.5) --ind;
    else ++ind;
    if (ind >= 0 && ind < trees.size()) {
        pcl::PointXYZ p2 = nearestInTree(trees, ind, p, radius);
        return dist2(p,p1)<dist2(p,p2)?p1:p2;
    }
    return p1;
}
PointXYZ ArrayMatrix::nearest(PointXYZ p, double radius) const
{
    if (!isValid(p)) return NaNPt;
    int qx = p.x>0?1:0;
    int qy = p.y>0?1:0;
    int xx = (abs(p.x/resolution) - 0.5);
    int yy = (abs(p.y/resolution) - 0.5);
    PointXYZ p1 = nearestInCell(qx,qy,xx,yy,p,radius);
    for (int x = 0; x < 2; ++x) {
        for (int y = 0; y < 2; ++y) {
            PointXYZ p2 = nearestInCell(qx,qy,xx+x,yy+y,p,radius);
            if (dist2(p,p2) < dist2(p,p1)) {
                p1 = p2;
            }
        }
    }
    return p1;
}

void markParallelPlanes(
        Vector3d v,
        PointCloud<PointXYZ>::ConstPtr cloud,
        vector<Vector4d>& planes,
        vector<int>& labels,
        int label)
{
    const double EDGE = -3;
    vector<bool> keep(planes.size(), true);
    for (int i = 0; i < planes.size(); ++i) {
        if (planes[i].head(3).dot(v) < sin(ANGLETHRESHOLD)) keep[i] = false;
    }
    vector<int> tmp(labels);
    markDepthDiscontinuities(cloud, DISCONTINUITYTHRESHOLD, tmp, EDGE, DISCONTINUITYBORDERWIDTH);
    for (int i = 0; i < labels.size(); ++i) {
        if (labels[i] > -1 && !keep[labels[i]] && tmp[i] != EDGE) labels[i] = label;
    }
}

void markPerpendicularPlanes(
        Vector3d v,
        PointCloud<PointXYZ>::ConstPtr cloud,
        vector<Vector4d>& planes,
        vector<int>& labels,
        int label)
{
    const double EDGE = -3;
    vector<bool> keep(planes.size(), true);
    for (int i = 0; i < planes.size(); ++i) {
        if (planes[i].head(3).dot(v) > cos(ANGLETHRESHOLD)) keep[i] = false;
    }
    vector<int> tmp(labels);
    markDepthDiscontinuities(cloud, DISCONTINUITYTHRESHOLD, tmp, EDGE, DISCONTINUITYBORDERWIDTH);
    for (int i = 0; i < labels.size(); ++i) {
        if (labels[i] > -1 && !keep[labels[i]] && tmp[i] != EDGE) labels[i] = label;
    }
}

void markEdges(
        PointCloud<PointXYZ>::ConstPtr cloud,
        vector<int>& labels,
        int label, int radius)
{
    for (int i = 0; i < cloud->height; ++i) {
        for (int j = 0; j < radius; ++j) {
            labels[i*cloud->width+j] = label;
            labels[(i+1)*cloud->width - 1 - j] = label;
        }
    }
    for (int j = 0; j < radius; ++j) {
        for (int i = 0; i < cloud->width; ++i) {
            labels[i + j*cloud->width] = label;
            labels[(cloud->height-1-j)*cloud->width+i] = label;
        }
    }
}
void markDepthDiscontinuities(
        PointCloud<PointXYZ>::ConstPtr cloud,
        double threshold,
        vector<int>& labels,
        int label, int radius)
{
    for (int i = radius; i < cloud->height-radius; ++i) {
        for (int j = radius; j < cloud->width-radius; ++j) {
            if (dist2(cloud->at(j,i), cloud->at(j+1,i)) > threshold*threshold ||
                dist2(cloud->at(j,i), cloud->at(j,i+1)) > threshold*threshold)
            {
                for (int x = -radius; x < radius; ++x) {
                    for (int y = -radius; y < radius; ++y) {
                        labels[(i+y+1)*cloud->width+j+x+1] = label;
                    }
                }
            }
        }
    }
}

void preprocessCloud(
        PointCloud<PointXYZ>::ConstPtr inputcloud,
        PointCloud<PointXYZ>::Ptr outputcloud,
        Matrix4d transform,
        vector<int>& ids,
        int id, bool removedepthdiscontinuities=true)
{
    transformPointCloud(*inputcloud, *outputcloud, transform);

    // Remove boundary points
    if (removedepthdiscontinuities) {
        markEdges(outputcloud, ids, id, EDGEREMOVAL);
    }
    filterLabelled(outputcloud, ids, id);
}

void computeCorrespondences(
        PointCloud<PointXYZ>::ConstPtr cloud,
        SearchStructure* tree,
        vector<PointXYZ>& correspondences)
{
    for (int i = 0; i < cloud->size(); ++i) {
        correspondences.push_back(tree->nearest(cloud->at(i),0.1));
    }
}

Matrix4d computeOptimalRigidXYTransform(vector<PointXYZ>& src, vector<PointXYZ>& tgt)
{
    // Compute centroids
    Vector2d cs(0,0);
    Vector2d ct(0,0);
    for (int i = 0; i < src.size(); ++i) {
        cs += Vector2d(src[i].x, src[i].y);
        ct += Vector2d(tgt[i].x, tgt[i].y);
    }
    cs *= 1./src.size();
    ct *= 1./src.size();

    // Compute centered matrices
    MatrixXd ms,mt;
    ms.resize(2,src.size());
    mt.resize(2,src.size());
    for (int i = 0; i < src.size(); ++i) {
        ms(0,i) = src[i].x - cs(0);
        ms(1,i) = src[i].y - cs(1);
        mt(0,i) = tgt[i].x - ct(0);
        mt(1,i) = tgt[i].y - ct(1);
    }

    MatrixXd S = ms*mt.transpose();
    // Get SVD of covariance matrix to solve for rotation
    JacobiSVD<MatrixXd> svd(S, ComputeFullU | ComputeFullV);
    Matrix2d rot = svd.matrixV()*svd.matrixU().transpose();
    if (rot.determinant() < 0) {
        rot(0,1) *= -1;
        rot(1,1) *= -1;
    }

    // Compute translation
    Vector2d trans = ct - rot*cs;

    // Convert back to 3d
    Matrix4d ret;
    ret.setIdentity();
    ret.topLeftCorner(2,2) = rot;
    ret.topRightCorner(2,1) = trans;
    return ret;
}

double filterCorrespondences(
        PointCloud<PointXYZ>::ConstPtr src,
        vector<PointXYZ>& corrs,
        vector<PointXYZ>& ptsrc,
        vector<PointXYZ>& pttgt,
        int targetnumber=0,
        double stddevthreshold=3)
{
    //double outlierprop = 0.05;   // Throw away this proportion of the furthest correspondences
    double maxcorrespondencedist = MAXCORRESPONDENCEDISTANCE;
    vector<pair<double, int> > dists;
    double meandist = 0;
    for (int i = 0; i < src->size(); ++i) {
        if (isValid(corrs[i])) {
            double d = dist2(src->at(i),corrs[i]);
            dists.push_back(make_pair(sqrt(d),i));
            meandist += d;
        }
    }
    meandist /= dists.size();
    double stdist = 0;
    for (int i = 0; i < dists.size(); ++i) {
        stdist += (dists[i].first-meandist)*(dists[i].first - meandist);
    }
    stdist /= dists.size();
    stdist = sqrt(stdist);

    sort(dists.begin(), dists.end());
    //int sz = dists.size()*(1-outlierprop);
    int sz = lower_bound(dists.begin(), dists.end(), make_pair(meandist + stddevthreshold*stdist,0)) - dists.begin();
    int sz2 = lower_bound(dists.begin(), dists.end(), make_pair(maxcorrespondencedist,0)) - dists.begin();
    if (sz2 < sz) sz = sz2;
    if (!sz) {
        cerr << "Error! No correspondences found!" << endl;
        return numeric_limits<double>::infinity();
    }
    if (!targetnumber) targetnumber = sz;
    int inc = sz/targetnumber;
    if (!inc) inc = 1;
    meandist = 0;
    int count = 0;
    for (int i = 0; i < sz; ++i) {
        meandist += dists[i].first*dists[i].first;
        ++count;
        if (i%inc == 0) {
            ptsrc.push_back(src->at(dists[i].second));
            pttgt.push_back(corrs[dists[i].second]);
        }
    }
    return sqrt(meandist/count);
}

double transformAngle(Matrix4d m) {
    return safe_acos((m.trace() - 2)/2);
}
double transformTranslation(Matrix4d m) {
    Vector3d x = m.topRightCorner(3,1);
    return x.norm();
}
bool converged(Matrix4d m, double dx, double dtheta) {
    return transformTranslation(m) < dx && transformAngle(m) < dtheta;
}

AlignmentResult alignPlaneToPlane(
        PointCloud<PointXYZ>::ConstPtr src,
        PointCloud<PointXYZ>::ConstPtr tgt,
        vector<Vector4d>& srcplanes, vector<int>& srcids,
        vector<Vector4d>& tgtplanes, vector<int>& tgtids,
        vector<int>& planecorrespondences,
        int maxiterations)
{
    // Put planes into correspondence
    int srcid;
    for (srcid = 0; srcid < planecorrespondences.size(); ++srcid) {
        if (planecorrespondences[srcid] > -1) break;
    }
    int tgtid = planecorrespondences[srcid];
    Matrix4d transform = overlapPlanes(srcplanes[srcid], tgtplanes[tgtid]);
    PointCloud<PointXYZ>::Ptr tsrc(new PointCloud<PointXYZ>);
    transformPointCloud(*src, *tsrc, transform);

    // Convert to common coordinate system
    PointCloud<PointXYZ>::Ptr ttgt(new PointCloud<PointXYZ>);
    Matrix4d coordtransform = overlapPlanes(tgtplanes[tgtid], Vector4d(0,0,1,0));
    transform = coordtransform*transform;

    // Filter clouds
    vector<int> fsrcids(srcids);
    vector<int> ftgtids(tgtids);
    markPerpendicularPlanes(srcplanes[srcid].head(3), tsrc, srcplanes, fsrcids, srcid);
    markPerpendicularPlanes(tgtplanes[tgtid].head(3), tgt, tgtplanes, ftgtids, tgtid);
    preprocessCloud(tgt, ttgt, coordtransform, ftgtids, tgtid, false);
    preprocessCloud(tsrc, tsrc, coordtransform, fsrcids, srcid);

    // Construct layered kdtrees for all non-plane points in tgt
    LayeredKdTrees lkdt(ttgt, SEARCHWIDTH);

    double error = numeric_limits<double>::infinity();
    for (int i = 0; i < maxiterations; ++i) {
        // Compute correspondences
        vector<PointXYZ> ptsrc;
        vector<PointXYZ> pttgt;
        vector<PointXYZ> corrs;
        computeCorrespondences(tsrc, &lkdt, corrs);
        double newerror = filterCorrespondences(tsrc, corrs, ptsrc, pttgt);
        if (!corrs.size()) return AlignmentResult(Matrix4d::Identity(), numeric_limits<double>::infinity());
        //if (abs(error-newerror) < errthreshold) break;
        error = newerror;

        // Compute rigid transform
        Matrix4d opt = computeOptimalRigidXYTransform(ptsrc, pttgt);
        cout << "Iteration " << i << ": RMSE " << newerror;
        cout << " Angle: " << transformAngle(opt);
        cout << " Translation: " << transformTranslation(opt) << endl;
        if (converged(opt, MINTRANSLATION, MINROTATION)) break;
        transform = opt*transform;
        transformPointCloud(*tsrc, *tsrc, opt);
    }
    transform = coordtransform.inverse()*transform;
    return AlignmentResult(transform, error);
}

AlignmentResult partialAlignPlaneToPlane(
        PointCloud<PointXYZ>::ConstPtr src,
        PointCloud<PointXYZ>::ConstPtr tgt,
        vector<Vector4d>& srcplanes, vector<int>& srcids, vector<int>& fsrcids,
        vector<Vector4d>& tgtplanes, vector<int>& tgtids, vector<int>& ftgtids,
        vector<int>& planecorrespondences,
        vector<PointXYZ>& pointcorrespondences,
        int ncorrs, double t)
{
    // Put planes into correspondence
    int srcid;
    for (srcid = 0; srcid < planecorrespondences.size(); ++srcid) {
        if (planecorrespondences[srcid] > -1) break;
    }
    int tgtid = planecorrespondences[srcid];
    Matrix4d transform = overlapPlanes(srcplanes[srcid], tgtplanes[tgtid]);
    PointCloud<PointXYZ>::Ptr tsrc(new PointCloud<PointXYZ>);
    transformPointCloud(*src, *tsrc, transform);

    // Convert to common coordinate system
    PointCloud<PointXYZ>::Ptr ttgt(new PointCloud<PointXYZ>);
    Matrix4d coordtransform = overlapPlanes(tgtplanes[tgtid], Vector4d(0,0,1,0));
    transform = coordtransform*transform;

    // Filter clouds
    fsrcids = srcids;
    ftgtids = tgtids;
    markPerpendicularPlanes(srcplanes[srcid].head(3), tsrc, srcplanes, fsrcids, srcid);
    markPerpendicularPlanes(tgtplanes[tgtid].head(3), tgt, tgtplanes, ftgtids, tgtid);
    preprocessCloud(tgt, ttgt, coordtransform, ftgtids, tgtid, false);
    preprocessCloud(tsrc, tsrc, coordtransform, fsrcids, srcid);

    // Construct layered kdtrees for all non-plane points in tgt
    LayeredKdTrees lkdt(ttgt, SEARCHWIDTH);

    vector<PointXYZ> ptsrc;
    vector<PointXYZ> pttgt;
    vector<PointXYZ> corrs;
    computeCorrespondences(tsrc, &lkdt, corrs);
    double error = filterCorrespondences(tsrc, corrs, ptsrc, pttgt, ncorrs, t);
    if (!corrs.size()) return AlignmentResult(Matrix4d::Identity(), numeric_limits<double>::infinity());
    cout << "Error: " << error << endl;
    for (int i = 0; i < ptsrc.size(); ++i) {
        Vector4d pt(ptsrc[i].x, ptsrc[i].y, ptsrc[i].z, 1);
        Vector4d corr(pttgt[i].x, pttgt[i].y, pttgt[i].z, 1);
        pt = coordtransform.inverse()*pt;
        corr = coordtransform.inverse()*corr;
        pointcorrespondences.push_back(PointXYZ(pt(0), pt(1), pt(2)));
        pointcorrespondences.push_back(PointXYZ(corr(0), corr(1), corr(2)));
    }

    Matrix4d opt = computeOptimalRigidXYTransform(ptsrc, pttgt);
    transform = opt*transform;

    transform = coordtransform.inverse()*transform;
    return AlignmentResult(transform, error);
}

double computeOptimal1d(vector<double>& x) {
    sort(x.begin(), x.end());
    double t = 0.01;
    double best = 0;
    int bestcount = 0;

    for (double d = 0; d < 2*t; d += t) {
        double lo = x[0] + d;
        int c = 0;
        for (int i = 0; i < x.size(); ++i) {
            while (lo + 2*t < x[i]) {
                if (c > bestcount) {
                    bestcount = c;
                    best = lo;
                }
                lo += 2*t;
                c = 0;
            }
            ++c;
        }
    }
    double tot = 0;
    int n = 0;
    vector<double>::iterator it = lower_bound(x.begin(), x.end(), best);
    for (;it != x.end(); ++it) {
        if (*it >= best+2*t) break;
        tot += *it;
        ++n;
    }
    return tot/n;
}

AlignmentResult alignEdgeToEdge(
        PointCloud<PointXYZ>::ConstPtr src,
        PointCloud<PointXYZ>::ConstPtr tgt,
        vector<Vector4d>& srcplanes, vector<int>& srcids,
        vector<Vector4d>& tgtplanes, vector<int>& tgtids,
        vector<int>& planecorrespondences,
        int maxiterations)
{
    // Put planes into correspondence
    vector<int> ids;
    for (int i = 0; i < planecorrespondences.size(); ++i) {
        if (planecorrespondences[i] > -1) ids.push_back(i);
    }
    Matrix4d transform = overlapEdge(srcplanes[ids[0]], srcplanes[ids[1]], tgtplanes[planecorrespondences[ids[0]]], tgtplanes[planecorrespondences[ids[1]]]);
    PointCloud<PointXYZ>::Ptr tsrc(new PointCloud<PointXYZ>);
    transformPointCloud(*src, *tsrc, transform);

    // Convert to common coordinate system
    PointCloud<PointXYZ>::Ptr ttgt(new PointCloud<PointXYZ>);
    Matrix4d coordtransform = overlapEdge(tgtplanes[planecorrespondences[ids[0]]],
                                          tgtplanes[planecorrespondences[ids[1]]],
                                          Vector4d(1,0,0,0),
                                          Vector4d(0,1,0,0));
    transform = coordtransform*transform;

    // Filter clouds
    vector<int> fsrcids(srcids);
    vector<int> ftgtids(tgtids);
    for (int i = 0; i < ftgtids.size(); ++i) {
        if (ftgtids[i] == planecorrespondences[ids[1]]) {
            ftgtids[i] = planecorrespondences[ids[0]];
        }
    }
    for (int i = 0; i < fsrcids.size(); ++i) {
        if (fsrcids[i] == ids[1]) {
            fsrcids[i] = ids[0];
        }
    }
    Vector3d srcax = srcplanes[ids[0]].head(3);
    srcax = srcax.cross((Vector3d) (srcplanes[ids[1]].head(3)));
    Vector3d tgtax = tgtplanes[planecorrespondences[ids[0]]].head(3);
    tgtax = tgtax.cross((Vector3d) (tgtplanes[planecorrespondences[ids[1]]].head(3)));
    markParallelPlanes(srcax, tsrc, srcplanes, fsrcids, ids[0]);
    markParallelPlanes(tgtax, tgt, tgtplanes, ftgtids, planecorrespondences[ids[0]]);
    preprocessCloud(tgt, ttgt, coordtransform, ftgtids, planecorrespondences[ids[0]], false);
    preprocessCloud(tsrc, tsrc, coordtransform, fsrcids, ids[0]);

    // Construct search structure
    ArrayMatrix am(ttgt, SEARCHWIDTH);

    double error = numeric_limits<double>::infinity();
    for (int i = 0; i < maxiterations; ++i) {
        // Compute correspondences
        vector<PointXYZ> ptsrc;
        vector<PointXYZ> pttgt;
        vector<PointXYZ> corrs;
        computeCorrespondences(tsrc, &am, corrs);
        double newerror = filterCorrespondences(tsrc, corrs, ptsrc, pttgt);
        if (!corrs.size()) return AlignmentResult(Matrix4d::Identity(), numeric_limits<double>::infinity());
        //if (abs(error-newerror) < errthreshold) break;
        error = newerror;

        // Compute rigid translation
        vector<double> dists;
        for (int i = 0; i < ptsrc.size(); ++i) {
            dists.push_back(ptsrc[i].z - pttgt[i].z);
        }
        Matrix4d transl;
        transl.setIdentity();
        transl(2,3) = -computeOptimal1d(dists);
        cout << "Iteration " << i << ": RMSE " << newerror;
        cout << " Translation: " << abs(transl(2,3)) << endl;
        if (converged(transl, MINTRANSLATION, MINROTATION)) break;
        transform = transl*transform;
        transformPointCloud(*tsrc, *tsrc, transl);
    }
    transform = coordtransform.inverse()*transform;
    return AlignmentResult(transform, error);
}
AlignmentResult partialAlignEdgeToEdge(
        PointCloud<PointXYZ>::ConstPtr src,
        PointCloud<PointXYZ>::ConstPtr tgt,
        vector<Vector4d>& srcplanes, vector<int>& srcids, vector<int>& fsrcids,
        vector<Vector4d>& tgtplanes, vector<int>& tgtids, vector<int>& ftgtids,
        vector<int>& planecorrespondences,
        vector<PointXYZ>& pointcorrespondences,
        int ncorrs, double t)
{
    // Put planes into correspondence
    vector<int> ids;
    for (int i = 0; i < planecorrespondences.size(); ++i) {
        if (planecorrespondences[i] > -1) ids.push_back(i);
    }
    Matrix4d transform = overlapEdge(srcplanes[ids[0]], srcplanes[ids[1]], tgtplanes[planecorrespondences[ids[0]]], tgtplanes[planecorrespondences[ids[1]]]);
    PointCloud<PointXYZ>::Ptr tsrc(new PointCloud<PointXYZ>);
    transformPointCloud(*src, *tsrc, transform);

    // Convert to common coordinate system
    PointCloud<PointXYZ>::Ptr ttgt(new PointCloud<PointXYZ>);
    Matrix4d coordtransform = overlapEdge(tgtplanes[planecorrespondences[ids[0]]],
                                          tgtplanes[planecorrespondences[ids[1]]],
                                          Vector4d(1,0,0,0),
                                          Vector4d(0,1,0,0));
    transform = coordtransform*transform;

    // Filter clouds
    fsrcids = srcids;
    ftgtids = tgtids;
    for (int i = 0; i < ftgtids.size(); ++i) {
        if (ftgtids[i] == planecorrespondences[ids[1]]) {
            ftgtids[i] = planecorrespondences[ids[0]];
        }
    }
    for (int i = 0; i < fsrcids.size(); ++i) {
        if (fsrcids[i] == ids[1]) {
            fsrcids[i] = ids[0];
        }
    }
    Vector3d srcax = srcplanes[ids[0]].head(3);
    srcax = srcax.cross((Vector3d) (srcplanes[ids[1]].head(3)));
    Vector3d tgtax = tgtplanes[planecorrespondences[ids[0]]].head(3);
    tgtax = tgtax.cross((Vector3d) (tgtplanes[planecorrespondences[ids[1]]].head(3)));
    markParallelPlanes(srcax, tsrc, srcplanes, fsrcids, ids[0]);
    markParallelPlanes(tgtax, tgt, tgtplanes, ftgtids, planecorrespondences[ids[0]]);
    preprocessCloud(tgt, ttgt, coordtransform, ftgtids, planecorrespondences[ids[0]], false);
    preprocessCloud(tsrc, tsrc, coordtransform, fsrcids, ids[0]);

    // Construct search structure
    ArrayMatrix am(ttgt, SEARCHWIDTH);
    // Compute correspondences
    vector<PointXYZ> ptsrc;
    vector<PointXYZ> pttgt;
    vector<PointXYZ> corrs;
    computeCorrespondences(tsrc, &am, corrs);
    double error = filterCorrespondences(tsrc, corrs, ptsrc, pttgt, ncorrs, t);
    if (!corrs.size()) return AlignmentResult(Matrix4d::Identity(), numeric_limits<double>::infinity());
    cout << "Error: " << error << endl;
    for (int i = 0; i < ptsrc.size(); ++i) {
        Vector4d pt(ptsrc[i].x, ptsrc[i].y, ptsrc[i].z, 1);
        Vector4d corr(pttgt[i].x, pttgt[i].y, pttgt[i].z, 1);
        pt = coordtransform.inverse()*pt;
        corr = coordtransform.inverse()*corr;
        pointcorrespondences.push_back(PointXYZ(pt(0), pt(1), pt(2)));
        pointcorrespondences.push_back(PointXYZ(corr(0), corr(1), corr(2)));
    }
    vector<double> dists;
    for (int i = 0; i < ptsrc.size(); ++i) {
        dists.push_back(ptsrc[i].z - pttgt[i].z);
    }
    Matrix4d transl;
    transl.setIdentity();
    transl(2,3) = -computeOptimal1d(dists);
    transform = coordtransform.inverse()*transl*transform;
    return AlignmentResult(transform, error);
}

AlignmentResult alignCornerToCorner(
        PointCloud<PointXYZ>::ConstPtr src,
        PointCloud<PointXYZ>::ConstPtr tgt,
        vector<Vector4d>& srcplanes, vector<int>& srcids,
        vector<Vector4d>& tgtplanes, vector<int>& tgtids,
        vector<int>& planecorrespondences)
{
    // Put planes into correspondence
    vector<int> ids;
    for (int i = 0; i < planecorrespondences.size(); ++i) {
        if (planecorrespondences[i] > -1) ids.push_back(i);
    }
    Matrix4d transform = overlapCorner(
            srcplanes[ids[0]],
            srcplanes[ids[1]],
            srcplanes[ids[2]],
            tgtplanes[planecorrespondences[ids[0]]],
            tgtplanes[planecorrespondences[ids[1]]],
            tgtplanes[planecorrespondences[ids[2]]]);
    PointCloud<PointXYZ>::Ptr tsrc(new PointCloud<PointXYZ>);
    transformPointCloud(*src, *tsrc, transform);

    // Convert to common coordinate system
    PointCloud<PointXYZ>::Ptr ttgt(new PointCloud<PointXYZ>);
    Matrix4d coordtransform = overlapEdge(tgtplanes[planecorrespondences[ids[0]]],
                                          tgtplanes[planecorrespondences[ids[1]]],
                                          Vector4d(1,0,0,0),
                                          Vector4d(0,1,0,0));

    // Filter clouds
    vector<int> fsrcids(srcids);
    vector<int> ftgtids(tgtids);
    for (int i = 0; i < ftgtids.size(); ++i) {
        if (ftgtids[i] == planecorrespondences[ids[1]]) {
            ftgtids[i] = planecorrespondences[ids[0]];
        }
    }
    for (int i = 0; i < fsrcids.size(); ++i) {
        if (fsrcids[i] == ids[1]) {
            fsrcids[i] = ids[0];
        }
    }
    Vector3d srcax = srcplanes[ids[0]].head(3);
    srcax = srcax.cross((Vector3d) (srcplanes[ids[1]].head(3)));
    Vector3d tgtax = tgtplanes[planecorrespondences[ids[0]]].head(3);
    tgtax = tgtax.cross((Vector3d) (tgtplanes[planecorrespondences[ids[1]]].head(3)));
    markParallelPlanes(srcax, tsrc, srcplanes, fsrcids, ids[0]);
    markParallelPlanes(tgtax, tgt, tgtplanes, ftgtids, planecorrespondences[ids[0]]);
    preprocessCloud(tgt, ttgt, coordtransform, ftgtids, planecorrespondences[ids[0]], false);
    preprocessCloud(tsrc, tsrc, coordtransform, fsrcids, ids[0]);

    // Construct search structure
    ArrayMatrix am(ttgt, SEARCHWIDTH);
    vector<PointXYZ> ptsrc;
    vector<PointXYZ> pttgt;
    vector<PointXYZ> corrs;
    computeCorrespondences(tsrc, &am, corrs);
    double err = filterCorrespondences(tsrc, corrs, ptsrc, pttgt);
    return AlignmentResult(transform, err);
}
