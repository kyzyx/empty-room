#include <pcl/conversions.h>

#include "orientation_finder.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>

using namespace std;
using namespace pcl;

OrientationFinder::OrientationFinder() :
    resolution(100),
    anglethreshold(M_PI/40)
{
}

OrientationFinder::OrientationFinder(pcl::PolygonMesh::Ptr m) :
    resolution(100),
    anglethreshold(M_PI/40),
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

bool OrientationFinder::computeOrientation()
{
    int numbins = 4*resolution*resolution;
    vector<double> totalarea(numbins,0);
    vector<vector<int> > histogram(numbins);
    // 1. Compute face normals
    if (facenormals.empty()) {
        if (!computeNormals()) return false;
    }

    // 2. Fill in histogram
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
        double theta = atan2(x,z);
        if (theta < 0) theta += 2*M_PI;
        double phi = acos(y);
        int thetaind = theta/M_PI*2*resolution;
        int phiind = 2*phi/M_PI*resolution;
        if (phiind == resolution) phiind--;
        int ind = thetaind*resolution + phiind;
        histogram[ind].push_back(i);
        totalarea[ind] += norm;
    }

    // 3. Find histogram peaks
    for (int a = 0; a < 3; ++a) {
        int maxbin = -1;
        double maxarea = 0;
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

            if (totalarea[i] > maxarea) {
                maxbin = i;
                maxarea = totalarea[i];
            }
        }

        // 4. Compute average normal in peak bin
        Eigen::Vector3f axis(0.,0.,0.);
        for (int i = 0; i < histogram[maxbin].size(); ++i) {
            if (facenormals[histogram[maxbin][i]](1) < 0) {
                axis -= facenormals[histogram[maxbin][i]];
            } else {
                axis += facenormals[histogram[maxbin][i]];
            }
        }
        axis = axis.normalized();
        axes.push_back(axis);
    }
    return true;
}
