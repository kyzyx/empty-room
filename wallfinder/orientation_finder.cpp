#include <pcl/conversions.h>
#include <pcl/common/transforms.h>

#include "orientation_finder.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>

using namespace std;
using namespace pcl;


OrientationFinder::OrientationFinder(pcl::PolygonMesh::Ptr m) :
    mesh(m),
    cloud(new pcl::PointCloud<PointNormal>())
{
}

bool OrientationFinder::computeNormals(bool ccw)
{
    // Initialize point cloud
    {
        PointCloud<PointXYZ> tmp;
        fromPCLPointCloud2(mesh->cloud, tmp);
        cloud->points.resize(tmp.size());
        for (size_t i = 0; i < tmp.size(); ++i) {
            cloud->points[i].x = tmp[i].x;
            cloud->points[i].y = tmp[i].y;
            cloud->points[i].z = tmp[i].z;
        }
    }
    facenormals.resize(mesh->polygons.size());

    // Iterate over faces
    for (int i = 0; i < mesh->polygons.size(); ++i) {
        if (mesh->polygons[i].vertices.size() != 3) {
            return false;
        }
        vector<uint32_t>& f = mesh->polygons[i].vertices;
        Eigen::Vector3f a(
                (*cloud)[f[0]].x - (*cloud)[f[1]].x,
                (*cloud)[f[0]].y - (*cloud)[f[1]].y,
                (*cloud)[f[0]].z - (*cloud)[f[1]].z
        );
        Eigen::Vector3f b(
                (*cloud)[f[2]].x - (*cloud)[f[1]].x,
                (*cloud)[f[2]].y - (*cloud)[f[1]].y,
                (*cloud)[f[2]].z - (*cloud)[f[1]].z
        );
        Eigen::Vector3f c = a.cross(b);
        if (ccw) c = -c;

        // Add to vertex normals
        for (int j = 0; j < 3; ++j) {
            (*cloud)[f[j]].normal_x += c(0);
            (*cloud)[f[j]].normal_y += c(1);
            (*cloud)[f[j]].normal_z += c(2);
        }

        // Update face normal
        facenormals[i] = c;
    }

    // Update vertex normals
    for (int i = 0; i < cloud->size(); ++i) {
        double norm =
            (*cloud)[i].normal_x * (*cloud)[i].normal_x +
            (*cloud)[i].normal_y * (*cloud)[i].normal_y +
            (*cloud)[i].normal_z * (*cloud)[i].normal_z;
        norm = sqrt(norm);
        if (norm > 0) {
            (*cloud)[i].normal_x /= norm;
            (*cloud)[i].normal_y /= norm;
            (*cloud)[i].normal_z /= norm;
        }
    }

    return true;
}

void OrientationFinder::addNormToHistogram(
        double x, double y, double z, int resolution, double weight)
{
    double theta = atan2(x,z);
    if (theta < 0) theta += 2*M_PI;
    double phi = acos(y);
    int thetaind = theta/M_PI*2*resolution;
    int phiind = 2*phi/M_PI*resolution;
    if (phiind == resolution) phiind--;
    int ind = thetaind*resolution + phiind;
    histogram[ind].push_back(Eigen::Vector3f(x,y,z));
    histogramweights[ind].push_back(weight);
    totalweight[ind] += weight;
}
bool OrientationFinder::computeOrientation(double anglethreshold, int resolution)
{
    int numbins = 4*resolution*resolution;
    totalweight.resize(numbins,0);
    histogram.resize(numbins);
    histogramweights.resize(numbins);
    if (!prepareComputeOrientation()) return false;
    fillHistogram(resolution);

    // Find histogram peaks
    for (int a = 0; a < 3; ++a) {
        int maxbin = -1;
        double maxweight = 0;
        for (int i = 0; i < numbins; ++i) {
            double theta = (i/resolution)*M_PI/(2*resolution);
            double phi = (i%resolution)*M_PI/(2*resolution);
            Eigen::Vector3f binvec(
                    sin(theta)*sin(phi),
                    cos(phi),
                    cos(theta)*sin(phi)
            );
            bool perpendicular = true;
            for (int j = 0; j < a; ++j) {
                double angle = acos(abs(axes[j].dot(binvec)));
                if (abs(angle - M_PI/2) > anglethreshold) {
                    perpendicular = false;
                    break;
                }
            }
            if (!perpendicular) continue;

            if (totalweight[i] > maxweight) {
                maxbin = i;
                maxweight = totalweight[i];
            }
        }

        // Compute average normal in peak bin
        Eigen::Vector3f axis(0.,0.,0.);
        for (int i = 0; i < histogram[maxbin].size(); ++i) {
            if (histogram[maxbin][i](1) < 0) {
                axis -= histogramweights[maxbin][i]*histogram[maxbin][i];
            } else {
                axis += histogramweights[maxbin][i]*histogram[maxbin][i];
            }
        }
        axis = axis.normalized();
        axes.push_back(axis);
    }
    return true;
}

void OrientationFinder::normalize()
{
    // Find axis closest to vertical
    for (int i = 1; i < 3; ++i) {
        if (axes[i](1) > axes[0](1)) swap(axes[i], axes[0]);
    }
    Eigen::Vector3f up = axes[0];
    Eigen::Vector3f right = axes[1];
    Eigen::Vector3f towards = right.cross(up);
    towards = towards.normalized();
    double d = M_PI/2 - acos(up.dot(right));
    // Ensure axes are perpendicular
    up = Eigen::AngleAxisf(d/2, towards)*up;
    right = Eigen::AngleAxisf(-d/2, towards)*right;
    // Rotate to align axes
    Eigen::Affine3f transform = getTransformationFromTwoUnitVectors(up, right);
    transformPointCloudWithNormals(*cloud, *cloud, transform);


    // Translate so that bounding box has minimum at (0,0,0)
    Eigen::Vector3f minimum(numeric_limits<float>::max(),
                            numeric_limits<float>::max(),
                            numeric_limits<float>::max());
    for (int i = 0; i < cloud->size(); ++i) {
        minimum(0) = min((*cloud)[i].x, minimum(0));
        minimum(1) = min((*cloud)[i].y, minimum(1));
        minimum(2) = min((*cloud)[i].z, minimum(2));
    }
    Eigen::Matrix4f trans;
    trans.setIdentity();
    trans.block<3,1>(0,3) = -minimum;
    transformPointCloudWithNormals(*cloud, *cloud, trans);
    normalizationtransform = trans*transform.matrix();
}
// Compute orientation with normals
bool NormalOrientationFinder::prepareComputeOrientation() {
    if (facenormals.empty()) {
        if (!computeNormals()) return false;
    }
    return true;
}
void NormalOrientationFinder::fillHistogram(int resolution) {
    for (int i = 0; i < mesh->polygons.size(); ++i) {
        double norm = facenormals[i].norm();
        if (norm == 0) continue;
        double x,y,z;
        if (facenormals[i](1) < 0) {
            x = -facenormals[i](0)/norm;
            y = -facenormals[i](1)/norm;
            z = -facenormals[i](2)/norm;
        } else {
            x = facenormals[i](0)/norm;
            y = facenormals[i](1)/norm;
            z = facenormals[i](2)/norm;
        }
        addNormToHistogram(x,y,z,resolution,norm);
    }
}
// End Compute orientation with normals
// Compute orientation with planes
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
void PlaneOrientationFinder::fillHistogram(int resolution) {
    ModelCoefficients coefficients;
    PointIndices::Ptr inliers (new PointIndices());
    SACSegmentation<PointNormal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(SACMODEL_PLANE);
    seg.setMethodType(SAC_RANSAC);
    seg.setDistanceThreshold(planeDistance);
    ExtractIndices<PointNormal> extract;
    int nr_points = cloud->points.size();
    PointCloud<PointNormal>::Ptr segmented_cloud(new PointCloud<PointNormal>(*cloud));
    PointCloud<PointNormal>::Ptr swap_cloud(new PointCloud<PointNormal>());
    while (segmented_cloud->points.size() > 0.1*nr_points) {
        seg.setInputCloud (segmented_cloud);
        seg.segment (*inliers, coefficients);
        if (inliers->indices.size () == 0) {
            break;
        }
        if (coefficients.values[1] < 0) {
            for (int i = 0; i < 4; ++i) {
                coefficients.values[i] = -coefficients.values[i];
            }
        }
        Eigen::Vector3f candidate(coefficients.values[0],
                                  coefficients.values[1],
                                  coefficients.values[2]);
        addNormToHistogram(candidate(0), candidate(1), candidate(2), resolution, inliers->indices.size());

        extract.setInputCloud(segmented_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*swap_cloud);
        segmented_cloud.swap(swap_cloud);
    }
}
// End Compute orientation with planes
