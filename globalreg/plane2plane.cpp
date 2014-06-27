#include "pairwise.h"
#include "plane2plane.h"
#include <pcl/point_representation.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>

using namespace pcl;
using namespace Eigen;
using namespace std;

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

static DefaultPointRepresentation<PointXYZ> pr;
inline double dist2(PointXYZ a, PointXYZ b) {
    if (!pr.isValid(a) || !pr.isValid(b)) return numeric_limits<double>::infinity();
    return (Vector3f(a.x,a.y,a.z)-Vector3f(b.x,b.y,b.z)).squaredNorm();
}
PointXYZ LayeredKdTrees::nearest(PointXYZ p, double radius=0.1)
{
    std::vector<Tree*>& trees = p.z>0?postrees:negtrees;
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

void markDepthDiscontinuities(
        PointCloud<PointXYZ>::ConstPtr cloud,
        double threshold,
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

    // Remove boundary points and wall points
    vector<int> prune(ids);
    if (removedepthdiscontinuities) {
        markDepthDiscontinuities(outputcloud, 0.1, prune, id, 2);
    }
    filterLabelled(outputcloud, prune, id);
}

void computeCorrespondences(
        PointCloud<PointXYZ>::ConstPtr cloud,
        LayeredKdTrees& tree,
        vector<PointXYZ>& correspondences)
{
    for (int i = 0; i < cloud->size(); ++i) {
        correspondences.push_back(tree.nearest(cloud->at(i),0.1));
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

void filterCorrespondences(
        PointCloud<PointXYZ>::ConstPtr src,
        vector<PointXYZ>& corrs,
        vector<PointXYZ>& ptsrc,
        vector<PointXYZ>& pttgt,
        int targetnumber=0,
        double stddevthreshold=3)
{
    //double outlierprop = 0.05;   // Throw away this proportion of the furthest correspondences
    vector<pair<double, int> > dists;
    double meandist = 0;
    for (int i = 0; i < src->size(); ++i) {
        if (pr.isValid(corrs[i])) {
            double d = dist2(src->at(i),corrs[i]);
            dists.push_back(make_pair(d,i));
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
    if (!targetnumber) targetnumber = sz;
    int inc = sz/targetnumber;
    if (!inc) inc = 1;
    for (int i = 0; i < sz; i += inc) {
        ptsrc.push_back(src->at(dists[i].second));
        pttgt.push_back(corrs[dists[i].second]);
    }
}

Matrix4d alignPlaneToPlane(
        PointCloud<PointXYZ>::ConstPtr src,
        PointCloud<PointXYZ>::ConstPtr tgt,
        vector<Vector4d>& srcplanes, vector<int>& srcids,
        vector<Vector4d>& tgtplanes, vector<int>& tgtids,
        vector<int>& planecorrespondences)
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
    preprocessCloud(tgt, ttgt, coordtransform, tgtids, tgtid, false);
    preprocessCloud(tsrc, tsrc, coordtransform, srcids, srcid, false);

    // Construct layered kdtrees for all non-plane points in tgt
    LayeredKdTrees lkdt(ttgt, 0.01);

    // Compute correspondences
    vector<PointXYZ> ptsrc;
    vector<PointXYZ> pttgt;
    vector<PointXYZ> corrs;
    computeCorrespondences(tsrc, lkdt, corrs);
    filterCorrespondences(tsrc, corrs, ptsrc, pttgt);

    // Compute rigid transform
    Matrix4d opt = computeOptimalRigidXYTransform(ptsrc, pttgt);
    transform = opt*transform;

    // FIXME: Iterate
    transform = coordtransform.inverse()*transform;
    return transform;
}

Matrix4d partialAlignPlaneToPlane(
        PointCloud<PointXYZ>::ConstPtr src,
        PointCloud<PointXYZ>::ConstPtr tgt,
        vector<Vector4d>& srcplanes, vector<int>& srcids,
        vector<Vector4d>& tgtplanes, vector<int>& tgtids,
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
    preprocessCloud(tgt, ttgt, coordtransform, tgtids, tgtid);
    preprocessCloud(tsrc, tsrc, coordtransform, srcids, srcid);

    // Construct layered kdtrees for all non-plane points in tgt
    LayeredKdTrees lkdt(ttgt, 0.01);

    vector<PointXYZ> ptsrc;
    vector<PointXYZ> pttgt;
    vector<PointXYZ> corrs;
    computeCorrespondences(tsrc, lkdt, corrs);
    filterCorrespondences(tsrc, corrs, ptsrc, pttgt, ncorrs, t);
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
    return transform;
}
