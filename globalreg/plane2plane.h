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

class SearchStructure {
    public:
        SearchStructure() {
            float nan = std::numeric_limits<float>::quiet_NaN();
            NaNPt = pcl::PointXYZ(nan, nan, nan);
        }
        virtual pcl::PointXYZ nearest(pcl::PointXYZ p, double radius) const = 0;
    protected:
        pcl::PointXYZ NaNPt;
};

class LayeredKdTrees : public SearchStructure {
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
        }
        ~LayeredKdTrees() { if (data) delete [] data; }

        virtual pcl::PointXYZ nearest(pcl::PointXYZ p, double radius) const;
    private:
        pcl::PointXYZ nearestInTree(const std::vector<Tree*>& trees, int ind, pcl::PointXYZ p, double radius) const {
            if (trees[ind]->size() == 0) return NaNPt;
            double query[2];
            query[0] = p.x; query[1] = p.y;
            FMat m(query, 1, 2);
            std::vector<std::vector<int> > indices;
            std::vector<std::vector<double> > dists;
            flann::SearchParams params(-1);
            params.max_neighbors = 1;
            int n = trees[ind]->radiusSearch(m, indices, dists, radius, params);
            if (n) {
                return pcl::PointXYZ(trees[ind]->getPoint(indices[0][0])[0], trees[ind]->getPoint(indices[0][0])[1], p.z);
            }
            else return NaNPt;
        }
        int len;
        double* data;
        double thickness;
        std::vector<Tree*> postrees;
        std::vector<Tree*> negtrees;
};

class ArrayMatrix : public SearchStructure{
    public:
        ArrayMatrix(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, double res=0.01)
            : resolution(res)
        {
            for (int i = 0; i < cloud->size(); ++i) {
                insert(cloud->at(i));
            }
        }

        void insert(pcl::PointXYZ p) {
            int qx = p.x>0?1:0;
            int qy = p.y>0?1:0;
            int xx = abs(p.x/resolution);
            int yy = abs(p.y/resolution);
            if (xx == 0 || yy == 0) return;
            if (xx >= data[qx][qy].size()) data[qx][qy].resize(xx+1);
            if (yy >= data[qx][qy][xx].size()) data[qx][qy][xx].resize(yy+1);
            data[qx][qy][xx][yy].insert(p.z);
        }
        virtual pcl::PointXYZ nearest(pcl::PointXYZ p, double radius) const;
    private:
        pcl::PointXYZ nearestInCell(int qx, int qy, int xx, int yy, pcl::PointXYZ p, double radius) const {
            if (xx < 0 || xx >= data[qx][qy].size()) return NaNPt;
            if (yy < 0 || yy >= data[qx][qy][xx].size()) return NaNPt;
            if (data[qx][qy][xx][yy].size() == 0) return NaNPt;
            std::set<double>::iterator lb, ub;
            ub = data[qx][qy][xx][yy].lower_bound(p.z);
            if (ub == data[qx][qy][xx][yy].begin()) return pcl::PointXYZ(p.x, p.y, *ub);
            lb = ub; lb--;
            if (ub == data[qx][qy][xx][yy].end()) return pcl::PointXYZ(p.x, p.y, *lb);
            if (p.z - *lb < *ub - p.z) return pcl::PointXYZ(p.x, p.y, *lb);
            else return pcl::PointXYZ(p.x, p.y, *ub);
        }
        double resolution;
        std::vector<std::vector<std::set<double> > > data[2][2];
};

void markDepthDiscontinuities(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        double threshold,
        std::vector<int>& labels,
        int label=1, int radius=4);

void computeCorrespondences(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        SearchStructure* tree,
        std::vector<pcl::PointXYZ>& correspondences);

Eigen::Matrix4d computeOptimalRigidXYTransform(
        std::vector<pcl::PointXYZ>& src,
        std::vector<pcl::PointXYZ>& tgt);

Eigen::Matrix4d partialAlignPlaneToPlane(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr src,
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr tgt,
        std::vector<Eigen::Vector4d>& srcplanes, std::vector<int>& srcids, std::vector<int>& fsrcids,
        std::vector<Eigen::Vector4d>& tgtplanes, std::vector<int>& tgtids, std::vector<int>& ftgtids,
        std::vector<int>& planecorrespondences,
        std::vector<pcl::PointXYZ>& pointcorrespondences,
        int ncorrs, double t);
Eigen::Matrix4d partialAlignEdgeToEdge(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr src,
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr tgt,
        std::vector<Eigen::Vector4d>& srcplanes, std::vector<int>& srcids, std::vector<int>& fsrcids,
        std::vector<Eigen::Vector4d>& tgtplanes, std::vector<int>& tgtids, std::vector<int>& ftgtids,
        std::vector<int>& planecorrespondences,
        std::vector<pcl::PointXYZ>& pointcorrespondences,
        int ncorrs, double t);

#endif
