#ifndef _PLANE2PLANE_H
#define _PLANE2PLANE_H
#include <flann/flann.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <limits>
#include <vector>

class LayeredKdTrees {
    public:
        typedef flann::Matrix<double> FMat;
        typedef flann::Index<flann::L2<double> > Tree;

        LayeredKdTrees(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, double layerheight=0.01)
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

            float nan = std::numeric_limits<float>::quiet_NaN();
            NaNPt = pcl::PointXYZ(nan, nan, nan);
        }
        ~LayeredKdTrees() { if (data) delete [] data; }

        pcl::PointXYZ nearest(pcl::PointXYZ p) {
            std::vector<Tree*>& trees = p.z>0?postrees:negtrees;
            int ind = p.z/thickness;
            if (ind > trees.size()) return NaNPt;
            double query[2];
            query[0] = p.x; query[1] = p.y;
            FMat m(query, 1, 2);
            std::vector<std::vector<int> > indices;
            std::vector<std::vector<double> > dists;
            double radius = 0.8;
            flann::SearchParams params(-1);
            params.max_neighbors = 1;
            int n = trees[ind]->radiusSearch(m, indices, dists, radius, params);
            // FIXME: Search both neighboring slices, not just one
            if (n) {
                return pcl::PointXYZ(trees[ind]->getPoint(indices[0][0])[0], trees[ind]->getPoint(indices[0][0])[1], p.z);
            }
            else return NaNPt;
        }

    private:
        void insertPoint(pcl::PointXYZ p) {
            FMat m(data + len, 1, 2);
            data[len++] = p.x;
            data[len++] = p.y;
            std::vector<Tree*>& trees = p.z>0?postrees:negtrees;
            int ind = p.z/thickness;
            while (ind > trees.size()) {
                trees.push_back(new Tree(FMat(), flann::KDTreeSingleIndexParams()));
            }
            trees[ind]->addPoints(m);
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
#endif
