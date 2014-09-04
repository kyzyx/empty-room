#include "util.h"
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_representation.h>
#include <pcl/features/integral_image_normal.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

using namespace pcl;
using namespace std;
using namespace Eigen;

void loadFile(string fname, PointCloud<PointXYZ>::Ptr cloud, bool smoothing) {
    io::loadPCDFile<PointXYZ>(fname, *cloud);
    if (smoothing) {
        FastBilateralFilter<PointXYZ> fbf;
        fbf.setSigmaS(5);
        fbf.setSigmaR(0.04);
        fbf.setInputCloud(cloud);
        fbf.applyFilter(*cloud);
    }
    for (int i = 0; i < cloud->size(); ++i) {
        if (isnan(cloud->at(i).x)) cloud->at(i).z = cloud->at(i).x;
    }
}

int main(int argc, char** argv) {
    if (argc != 4) {
        cout << "Usage: postprocess MANIFEST XFORMPATTERN outputfile" << endl;
    }
    ifstream inf(argv[1]);
    string s;
    getline(inf,s);
    string xformpattern = argv[2]; string output = argv[3];

    ofstream points(output + ".txt");
    ofstream cameras(output + ".cam");

    Matrix4d cumXform = Matrix4d::Identity();
    char tmp[20];
    DefaultPointRepresentation<PointNormal> pr;
    for (int i = 0; !inf.eof(); ++i) {
        PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
        loadFile(s, cloud, false);
        getline(inf,s);

        // Compute normals
        PointCloud<PointNormal>::Ptr cloudn(new PointCloud<PointNormal>);
        copyPointCloud(*cloud, *cloudn);
        IntegralImageNormalEstimation<PointXYZ, PointNormal> ne;
        ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
        ne.setMaxDepthChangeFactor(0.05f);
        ne.setNormalSmoothingSize(12.0f);
        ne.setInputCloud(cloud);
        ne.compute(*cloudn);

        // Transform cloud
        sprintf(tmp, (xformpattern + ".xform").c_str(), i);
        Matrix4d t;
        ifstream in(tmp);
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 4; ++k) {
                in >> t(j,k);
            }
        }
        in.close();
        cumXform = cumXform*t;
        transformPointCloudWithNormals(*cloudn, *cloudn, cumXform);

        // Output camera params
        //    camera position
        cameras << cumXform(0,3) << " ";
        cameras << cumXform(1,3) << " ";
        cameras << cumXform(2,3) << " ";
        //    camera up
        cameras << -cumXform(0,1) << " ";
        cameras << -cumXform(1,1) << " ";
        cameras << -cumXform(2,1) << " ";
        //    camera towards
        cameras << cumXform(0,2) << " ";
        cameras << cumXform(1,2) << " ";
        cameras << cumXform(2,2) << endl;

        // Append points to file
        int subsample = 3;
        for (int j = 0; j < cloudn->height; j += subsample) {
            for (int k = 0; k < cloudn->width; k += subsample) {
                if (pr.isValid(cloudn->at(k,j)) && !isnan(cloudn->at(k,j).normal_x)) {
                    points << cloudn->at(k,j).x << " ";
                    points << cloudn->at(k,j).y << " ";
                    points << cloudn->at(k,j).z << " ";
                    points << cloudn->at(k,j).normal_x << " ";
                    points << cloudn->at(k,j).normal_y << " ";
                    points << cloudn->at(k,j).normal_z << endl;
                }
            }
        }
        cout << "Done processing frame " << i << endl;
    }
}
