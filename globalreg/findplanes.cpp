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

using namespace std;
using namespace Eigen;
using namespace pcl;

double calculateEdgeProportion(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<int> indices)
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
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
            if (edgeprop < 0.04) {
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        std::vector<Eigen::Vector4d>& planes, std::vector<int>& ids)
{
}

void findPlanes(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        std::vector<Eigen::Vector4d>& planes, std::vector<int>& ids)
{
    ids.resize(cloud->size());
    for (int i = 0; i < ids.size(); ++i) ids[i] = -1;
    return findPlanesRANSAC(cloud, planes, ids);
}
