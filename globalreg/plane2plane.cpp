#include "pairwise.h"
#include <pcl/point_representation.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <flann/flann.hpp>

using namespace pcl;
using namespace Eigen;
using namespace std;

class LayeredKdTrees {
    public:
        typedef flann::Matrix<double> FMat;
        typedef flann::Index<flann::L2<double> > Tree;

        LayeredKdTrees(PointCloud<PointXYZ>::ConstPtr cloud, double layerheight=0.01)
            : thickness(layerheight), len(0)
        {
            data = new double[cloud->size()*2];
            for (int i = 0; i < cloud->size(); ++i) insertPoint(cloud->at(i));
            for (int i = 0; i < postrees.size(); ++i) {
                postrees[i]->buildIndex();
            }
            for (int i = 0; i < negtrees.size(); ++i) {
                negtrees[i]->buildIndex();
            }

            float nan = numeric_limits<float>::quiet_NaN();
            NaNPt = PointXYZ(nan, nan, nan);
        }
        ~LayeredKdTrees() { if (data) delete [] data; }

        PointXYZ nearest(PointXYZ p) {
            vector<Tree*>& trees = p.z>0?postrees:negtrees;
            int ind = p.z/thickness;
            if (ind > trees.size()) return NaNPt;
            double query[2];
            query[0] = p.x; query[1] = p.y;
            FMat m(query, 1, 2);
            vector<vector<int> > indices;
            vector<vector<double> > dists;
            double radius = 0.8;
            flann::SearchParams params(-1);
            params.max_neighbors = 1;
            int n = trees[ind]->radiusSearch(m, indices, dists, radius, params);
            // FIXME: Search both neighboring slices, not just one
            if (n) {
                return PointXYZ(trees[ind]->getPoint(indices[0][0])[0], trees[ind]->getPoint(indices[0][0])[1], p.z);
            }
            else return NaNPt;
        }

    private:
        void insertPoint(PointXYZ p) {
            FMat m(data + len, 1, 2);
            data[len++] = p.x;
            data[len++] = p.y;
            vector<Tree*>& trees = p.z>0?postrees:negtrees;
            int ind = p.z/thickness;
            while (ind > trees.size()) {
                trees.push_back(new Tree(FMat(), flann::KDTreeSingleIndexParams()));
            }
            trees[ind]->addPoints(m);
        }

        PointXYZ NaNPt;
        int len;
        double* data;
        double thickness;
        vector<Tree*> postrees;
        vector<Tree*> negtrees;
};

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
