#include "findplanes.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>

const double ANGLETHRESHOLD = M_PI/9;
const double NOISETHRESHOLD = 0.01;
const double MININLIERPROPORTION = 0.07;
const double MAXEDGEPROPORTION = 0.04;

using namespace std;
using namespace Eigen;
using namespace pcl;

double calculateEdgeProportion(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, vector<int> indices)
{
    vector<uint8_t> labels(cloud->size(), -1);
    for (int i = 0; i < indices.size(); ++i) {
        int x = indices[i]%cloud->width;
        int y = indices[i]/cloud->width;

        labels[indices[i]] = 0;
        if (x != cloud->width-1 && labels[indices[i]+1])
            labels[indices[i]+1] = 1;
        if (x && labels[indices[i]-1])
            labels[indices[i]-1] = 1;
        if (y != cloud->height-1 && labels[indices[i]+cloud->width])
            labels[indices[i]+cloud->width] = 1;
        if (y && labels[indices[i]-cloud->width])
            labels[indices[i]-cloud->width] = 1;
    }
    int count = 0;
    for (int i = 0; i < labels.size(); ++i) {
        if (labels[i] == 1) ++count;
    }
    return count/(double) indices.size();
}
void findPlanesRANSAC(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        std::vector<Eigen::Vector4d>& planes, std::vector<int>& ids)
{
    ModelCoefficients::Ptr coef(new ModelCoefficients);
    PointIndices::Ptr inliers(new PointIndices);
    SACSegmentation<PointXYZL> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(SACMODEL_PLANE);
    seg.setMethodType(SAC_RANSAC);
    seg.setDistanceThreshold(2*NOISETHRESHOLD);
    seg.setMaxIterations(100);
    ExtractIndices<PointXYZL> extract;

    PointCloud<PointXYZL>::Ptr filtered(new PointCloud<PointXYZL>);
    copyPointCloud(*cloud, *filtered);
    for (int i = 0; i < filtered->size(); ++i) {
        filtered->at(i).label = i;
    }

    int n = 0;
    for (int z = 0; z < 10 && filtered->size() > MININLIERPROPORTION*cloud->size(); ++z) {
        // Find planes, if any
        seg.setInputCloud(filtered);
        seg.segment(*inliers, *coef);
        if (inliers->indices.size() == 0) {
            break;
        }
        // Filter planes
        // A plane is valid if it takes up a large proportion of the points
        // in the cloud, and if it has many large contiguous sections.
        if (inliers->indices.size() > MININLIERPROPORTION*cloud->size()) {
            vector<int> originalindices;
            for (int i = 0; i < inliers->indices.size(); ++i) {
                originalindices.push_back(filtered->at(inliers->indices[i]).label);
            }
            double edgeprop = calculateEdgeProportion(cloud, originalindices);
            if (edgeprop < MAXEDGEPROPORTION) {
                // Add plane to list
                for (int i = 0; i < originalindices.size(); ++i) {
                    ids[originalindices[i]] = n;
                }
                Vector4d p(coef->values[0], coef->values[1], coef->values[2], coef->values[3]);
                planes.push_back(p);
                // Remove points on plane
                extract.setInputCloud(filtered);
                extract.setIndices(inliers);
                extract.setNegative(true);
                extract.filter(*filtered);
                n++;
            }
        }
    }
}

void findPlanesWithNormals(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        std::vector<Eigen::Vector4d>& planes, std::vector<int>& ids)
{
    ModelCoefficients::Ptr coef(new ModelCoefficients);
    PointIndices::Ptr inliers(new PointIndices);
    SACSegmentationFromNormals<PointXYZINormal, PointXYZINormal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(SACMODEL_NORMAL_PLANE);
    seg.setMethodType(SAC_RANSAC);
    seg.setDistanceThreshold(NOISETHRESHOLD);
    seg.setMaxIterations(100);
    seg.setEpsAngle(ANGLETHRESHOLD);
    ExtractIndices<PointXYZINormal> extract;

    PointCloud<PointXYZINormal>::Ptr filtered(new PointCloud<PointXYZINormal>);
    copyPointCloud(*cloud, *filtered);
    // HACK: Use intensity (float field) as an id (int) field
    for (int i = 0; i < filtered->size(); ++i) {
        int* I = (int*) &(filtered->at(i).intensity);
        *I = i;
    }
    IntegralImageNormalEstimation<PointXYZ, PointXYZINormal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.05f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*filtered);

    int n = 0;
    while (filtered->size() > 0.2*cloud->size()) {
        // Find planes, if any
        seg.setInputCloud(filtered);
        seg.setInputNormals(filtered);
        seg.segment(*inliers, *coef);
        if (inliers->indices.size() == 0) {
            break;
        }

        // Add plane to list
        for (int i = 0; i < inliers->indices.size(); ++i) {
            int* I = (int*) &(filtered->at(inliers->indices[i]).intensity);
            int idx = *I;
            ids[idx] = n;
        }
        Vector4d p(coef->values[0], coef->values[1], coef->values[2], coef->values[3]);
        planes.push_back(p);

        // Remove points on plane
        extract.setInputCloud(filtered);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*filtered);
        n++;
    }
}

void findPlanesManhattan(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        std::vector<Eigen::Vector4d>& planes, std::vector<int>& ids)
{
}

void findPlanesGradient(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        std::vector<Eigen::Vector4d>& planes, std::vector<int>& ids)
{
    if (cloud->height == 1) {
        cerr << "Error: findPlanesGradient expects a range image" << endl;
        return;
    }
    // Calculate gradients
    double** dx = new double*[cloud->height];
    double** dy = new double*[cloud->height];
    for (int i = 0; i < cloud->height; ++i) {
        dx[i] = new double[cloud->width];
        dy[i] = new double[cloud->width];

        for (int j = 0; j < cloud->width; ++j) {
            if (j < cloud->width-1) dx[i][j] = cloud->at(j+1,i).z - cloud->at(j,i).z;
            if (i < cloud->height-1) dy[i][j] = cloud->at(j,i+1).z - cloud->at(j,i).z;
        }
    }
    // BFS to find components in image
    int n = 0;
    vector<int> sizes;
    PointRepresentation<PointXYZ> pr;
    for (int i = 0; i < cloud->height-1; ++i) {
        for (int j = 0; j < cloud->width-1; ++j) {
            if (ids[i*cloud->width + j] >= 0) continue;
            queue<int> nx; queue<int> ny;
            nx.push(j); ny.push(i);
            double adx = 0;
            double ady = 0;
            int cnt = 0;
            while (!nx.empty()) {
                int x = nx.top(); nx.pop();
                int y = ny.top(); ny.pop();
                if (!pr.isValid(cloud->at(x,y))) continue;
                ids[y*cloud->width + x] = n;
                adx = (adx*cnt + dx[x][y])/(cnt+1);
                ady = (ady*cnt + dy[x][y])/(cnt+1);
                ++cnt;
                for (int xx = x-1; xx <= x+1; ++xx) {
                    if (xx < 0 || xx >= cloud->width-1) continue;
                    int t = xx?0:1;
                    for (int yy = y-t; yy <= y+t; ++yy) {
                        if (yy < 0 || yy >= cloud->height-1) continue;
                        if (t) { // Check y gradients
                            if (abs(dy[xx][yy] - ady) > DGRAD) continue;
                        } else { // Check x gradients
                            if (!yy) continue;
                            if (abs(dx[xx][yy] - adx) > DGRAD) continue;
                        }
                        nx.push(xx); ny.push(yy);
                    }
                }
            }
            if (cnt) {
                size.push_back(cnt);
                n++;
            }
        }
    }

    // Global consistency
    for (int i = 0; i < n; ++i) {

    }

    // Filter out small components
    int nremoved = 0;
    for (int i = 0; i < n; ++i) {
        if (sizes[i+nremoved]/(double) cloud->size() < MININLIERPROPORTION) {
            nremoved++;
            for (int j = 0; j < ids.size(); ++j) {
                if (ids[j] == i) ids[j] = -1;
                else if (ids[j] > i) ids[j]--;
            }
            --i;
        }
    }
}

void findPlanes(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
        std::vector<Eigen::Vector4d>& planes, std::vector<int>& ids)
{
    ids.resize(cloud->size());
    for (int i = 0; i < ids.size(); ++i) ids[i] = -1;
    return findPlanesRANSAC(cloud, planes, ids);
}
