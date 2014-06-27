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

    // FIXME: Find correspondences
    // TODO: Consider outlier rejection
    // FIXME: Construct LSQ matrix for minimization and transform
    // Iterate
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

    vector<PointXYZ> ptcorrs;
    computeCorrespondences(tsrc, lkdt, ptcorrs);
    for (int i = 0; i < tsrc->size(); i += 1+tsrc->size()/ncorrs) {
        while (i < tsrc->size() && !pr.isValid(ptcorrs[i])) ++i;
        if (i == tsrc->size()) break;
        Vector4d pt(tsrc->at(i).x, tsrc->at(i).y, tsrc->at(i).z, 1);
        Vector4d corr(ptcorrs[i].x, ptcorrs[i].y, ptcorrs[i].z, 1);
        pt = coordtransform.inverse()*pt;
        corr = coordtransform.inverse()*corr;
        pointcorrespondences.push_back(PointXYZ(pt(0), pt(1), pt(2)));
        pointcorrespondences.push_back(PointXYZ(corr(0), corr(1), corr(2)));
    }
    // TODO: Consider outlier rejection
    // FIXME: Construct LSQ matrix for minimization and transform
    // Iterate
    transform = coordtransform.inverse()*transform;
    return transform;
}
