#include "pairwise.h"
#include "findplanes.h"
#include <pcl/io/io.h>
#include <pcl/io/grabber.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni2/openni.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;
using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
    if (argc < 3) {
        cout << "Usage: align cloud1.pcd cloud2.pcd" << endl;
        return 0;
    }
    visualization::PCLVisualizer viewer("Cloud viewer");
    PointCloud<PointXYZ>::Ptr cloud1(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr cloud2(new PointCloud<PointXYZ>);
    io::loadPCDFile<PointXYZ>(argv[1], *cloud1);
    io::loadPCDFile<PointXYZ>(argv[2], *cloud2);
    vector<Vector4d> srcplanes, tgtplanes;
    vector<int> srcids, tgtids;
    cout << "Finding planes..." << endl;
    findPlanes(cloud1, srcplanes, srcids);
    cout << "Done finding frame 1 planes" << endl;
    findPlanes(cloud2, tgtplanes, tgtids);
    cout << "Done finding planes" << endl;
    vector<int> planecorrespondences;

    int numcorrespondences = findPlaneCorrespondences(cloud1, cloud2, srcplanes, srcids, tgtplanes, tgtids, planecorrespondences);
    cout << numcorrespondences << endl;

    PointCloud<PointXYZRGB>::Ptr colored1(new PointCloud<PointXYZRGB>);
    copyPointCloud(*cloud1, *colored1);
    for (int i = 0; i < srcids.size(); ++i) {
        if (srcids[i] == -1) {
            colored1->at(i).r = 255;
            colored1->at(i).g = 255;
            colored1->at(i).b = 255;
        } else {
            colored1->at(i).r = 0;
            colored1->at(i).g = 0;
            colored1->at(i).b = 0;
            if (planecorrespondences[srcids[i]] == -1) {
                colored1->at(i).b = 255;
                if ((srcids[i]+1)&1) colored1->at(i).r = 127;
                if ((srcids[i]+1)&2) colored1->at(i).g = 127;
            } else {
                if ((planecorrespondences[srcids[i]]+1)&1) colored1->at(i).r = 255;
                if ((planecorrespondences[srcids[i]]+1)&2) colored1->at(i).g = 255;
                if ((planecorrespondences[srcids[i]]+1)&4) colored1->at(i).b = 255;
            }
        }
    }
    PointCloud<PointXYZRGB>::Ptr colored2(new PointCloud<PointXYZRGB>);
    copyPointCloud(*cloud2, *colored2);
    for (int i = 0; i < tgtids.size(); ++i) {
        if (tgtids[i] == -1) {
            colored2->at(i).r = 127;
            colored2->at(i).g = 127;
            colored2->at(i).b = 127;
        } else {
            if ((tgtids[i]+1)&1) colored2->at(i).r = 127;
            if ((tgtids[i]+1)&2) colored2->at(i).g = 127;
            colored2->at(i).b = 0;
        }
    }
    visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb1(colored1);
    visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb2(colored2);
    viewer.addPointCloud<PointXYZRGB>(colored1, rgb1, "Mesh1");
    viewer.addPointCloud<PointXYZRGB>(colored2, rgb2, "Mesh2");

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }
    return 0;
}
