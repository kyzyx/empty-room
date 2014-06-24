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
const double NOISETHRESHOLD = 0.008;


using namespace std;
using namespace Eigen;
using namespace pcl;

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
    while (filtered->size() > 0.2*cloud->size()) {
        // Find planes, if any
        seg.setInputCloud(filtered);
        seg.segment(*inliers, *coef);
        if (inliers->indices.size() == 0) {
            break;
        }

        // Add plane to list
        for (int i = 0; i < inliers->indices.size(); ++i) {
            int idx = filtered->at(inliers->indices[i]).label;
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
    return findPlanesWithNormals(cloud, planes, ids);
    //return findPlanesRANSAC(cloud, planes, ids);
}
