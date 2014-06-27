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

DefaultPointRepresentation<PointXYZ> pr;
inline double dist2(PointXYZ a, PointXYZ b) {
    if (!pr.isValid(a) || !pr.isValid(b)) return numeric_limits<double>::infinity();
    return (Vector3f(a.x,a.y,a.z)-Vector3f(b.x,b.y,b.z)).squaredNorm();
}
void markDepthDiscontinuities(
        PointCloud<PointXYZ>::ConstPtr cloud,
        double threshold,
        vector<int>& labels,
        int label=1)
{
    for (int i = 0; i < cloud->height; ++i) {
        labels[i*cloud->width] = label;
        labels[(i+1)*cloud->width - 1] = label;
    }
    for (int i = 0; i < cloud->width; ++i) {
        labels[i] = label;
        labels[(cloud->height-1)*cloud->width+i] = label;
    }
    for (int i = 1; i < cloud->height-1; ++i) {
        for (int j = 1; j < cloud->width-1; ++j) {
            if (dist2(cloud->at(j,i), cloud->at(j+1,i)) > threshold*threshold) {
                labels[i*cloud->width+j] = label;
                labels[i*cloud->width+j+1] = label;
            }
            if (dist2(cloud->at(j,i), cloud->at(j,i+1)) > threshold*threshold) {
                labels[i*cloud->width+j] = label;
                labels[(i+1)*cloud->width+j] = label;
            }
        }
    }
}

void preprocessCloud(
        PointCloud<PointXYZ>::ConstPtr inputcloud,
        PointCloud<PointXYZ>::Ptr outputcloud,
        Matrix4d transform,
        vector<int>& ids,
        int id)
{
    transformPointCloud(*inputcloud, *outputcloud, transform);

    // Remove boundary points and wall points
    vector<int> prune(ids);
    markDepthDiscontinuities(outputcloud, 0.1, prune, id);
    filterLabelled(outputcloud, prune, id);
}

void computeCorrespondences(
        PointCloud<PointXYZ>::ConstPtr cloud,
        LayeredKdTrees& tree,
        vector<PointXYZ>& correspondences)
{
    for (int i = 0; i < cloud->size(); ++i) {
        correspondences.push_back(tree.nearest(cloud->at(i),2.0));
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
        int targetnumber=0)
{
    double outlierprop = 0.01;   // Throw away this proportion of the furthest correspondences
    vector<pair<double, int> > dists;
    for (int i = 0; i < src->size(); ++i) {
        if (pr.isValid(corrs[i])) {
            dists.push_back(make_pair(dist2(src->at(i),corrs[i]),i));
        }
    }
    sort(dists.begin(), dists.end());
    int sz = dists.size()*(1-outlierprop);
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
    preprocessCloud(tgt, ttgt, coordtransform, tgtids, tgtid);
    preprocessCloud(tsrc, tsrc, coordtransform, srcids, srcid);

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
        int ncorrs)
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
    filterCorrespondences(tsrc, corrs, ptsrc, pttgt, ncorrs);
    for (int i = 0; i < ptsrc.size(); ++i) {
        Vector4d pt(ptsrc[i].x, ptsrc[i].y, ptsrc[i].z, 1);
        Vector4d corr(pttgt[i].x, pttgt[i].y, pttgt[i].z, 1);
        pt = coordtransform.inverse()*pt;
        corr = coordtransform.inverse()*corr;
        pointcorrespondences.push_back(PointXYZ(pt(0), pt(1), pt(2)));
        pointcorrespondences.push_back(PointXYZ(corr(0), corr(1), corr(2)));
    }
    transform = coordtransform.inverse()*transform;
    return transform;
}
