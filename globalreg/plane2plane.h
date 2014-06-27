#ifndef _PLANE2PLANE_H
#define _PLANE2PLANE_H
#include <flann/flann.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <limits>
#include <vector>

inline bool _PointXYZ_zcmp(pcl::PointXYZ a, pcl::PointXYZ b) {
    return a.z < b.z;
}
class LayeredKdTrees {
    public:
        typedef flann::Matrix<double> FMat;
        typedef flann::Index<flann::L2<double> > Tree;

        LayeredKdTrees(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, double layerheight=0.01)
            : thickness(layerheight), len(0)
        {
            data = new double[cloud->size()*2];

            std::vector<pcl::PointXYZ> sorted(cloud->size());
            for (int i = 0; i < cloud->size(); ++i) {
                sorted[i] = cloud->at(i);
            }
            sort(sorted.begin(), sorted.end(), _PointXYZ_zcmp);

            int lastind = abs(sorted[0].z/thickness);
            int start = 0;
            for (int i = 0; i < sorted.size(); ++i) {
                int ind = abs(sorted[i].z/thickness);
                if (ind != lastind) {
                    lastind = ind;
                    std::vector<Tree*>& trees = sorted[i].z>0?postrees:negtrees;
                    while (ind >= trees.size()) {
                        trees.push_back(new Tree(FMat(data,0,2), flann::KDTreeSingleIndexParams()));
                    }
                    FMat m(data + start, (len-start)/2, 2);
                    trees[ind]->addPoints(m);
                    start = len;
                }
                data[len++] = sorted[i].x;
                data[len++] = sorted[i].y;
            }

            for (int i = 0; i < postrees.size(); ++i) {
                if (postrees[i]->size()) postrees[i]->buildIndex();
            }
            for (int i = 0; i < negtrees.size(); ++i) {
                if (negtrees[i]->size()) negtrees[i]->buildIndex();
            }

            float nan = std::numeric_limits<float>::quiet_NaN();
            NaNPt = pcl::PointXYZ(nan, nan, nan);
        }
        ~LayeredKdTrees() { if (data) delete [] data; }

        pcl::PointXYZ nearest(pcl::PointXYZ p, double radius=0.1) {
            std::vector<Tree*>& trees = p.z>0?postrees:negtrees;
            int ind = abs(p.z/thickness);
            if (ind >= trees.size() || trees[ind]->size() == 0) return NaNPt;
            pcl::PointXYZ p1 = nearestInTree(trees, ind, p, radius);
            /*if (abs(p.z/thickness) - ind < 0.5) --ind;
            else ++ind;
            if (ind >= 0 && ind < trees.size()) {
                PointXYZ p2 = nearestInTree(trees, ind, p, radius);
                return dist2(p,p1)<dist2(p,p2)?p1:p2;
            }*/
            return p1;
        }

    private:
        pcl::PointXYZ nearestInTree(std::vector<Tree*>& trees, int ind, pcl::PointXYZ p, double radius) {
            double query[2];
            query[0] = p.x; query[1] = p.y;
            FMat m(query, 1, 2);
            std::vector<std::vector<int> > indices;
            std::vector<std::vector<double> > dists;
            flann::SearchParams params(-1);
            params.max_neighbors = 1;
            int n = trees[ind]->radiusSearch(m, indices, dists, radius, params);
            // FIXME: Search both neighboring slices, not just one
            if (n) {
                return pcl::PointXYZ(trees[ind]->getPoint(indices[0][0])[0], trees[ind]->getPoint(indices[0][0])[1], p.z);
            }
            else return NaNPt;
        }
        pcl::PointXYZ NaNPt;
        int len;
        double* data;
        double thickness;
        std::vector<Tree*> postrees;
        std::vector<Tree*> negtrees;
};

void markDepthDiscontinuities(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        double threshold,
        std::vector<int>& labels,
        int label);

void computeCorrespondences(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        LayeredKdTrees& tree,
        std::vector<pcl::PointXYZ>& correspondences);

Eigen::Matrix4d computeOptimalRigidXYTransform(
        std::vector<pcl::PointXYZ>& src,
        std::vector<pcl::PointXYZ>& tgt);

Eigen::Matrix4d partialAlignPlaneToPlane(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr src,
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr tgt,
        std::vector<Eigen::Vector4d>& srcplanes, std::vector<int>& srcids,
        std::vector<Eigen::Vector4d>& tgtplanes, std::vector<int>& tgtids,
        std::vector<int>& planecorrespondences,
        std::vector<pcl::PointXYZ>& pointcorrespondences,
        int ncorrs);

#endif
