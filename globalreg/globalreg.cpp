#include "pairwise.h"
#include "findplanes.h"
#include <pcl/io/io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;
using namespace std;
using namespace Eigen;

void showColoredCloud(visualization::PCLVisualizer& viewer, PointCloud<PointXYZ>::Ptr cloud, vector<int>& ids) {
    static char name[] = {'M','e','s','h','0','\0'};
    PointCloud<PointXYZRGB>::Ptr c(new PointCloud<PointXYZRGB>);
    copyPointCloud(*cloud, *c);
    for (int i = 0; i < ids.size(); ++i) {
        if (ids[i] == -1) {
            c->at(i).r = 255;
            c->at(i).g = 255;
            c->at(i).b = 255;
        } else if (ids[i] == 0) {
            c->at(i).r = 255;
            c->at(i).g = 0;
            c->at(i).b = 0;
        } else if (ids[i] == 1) {
            c->at(i).r = 0;
            c->at(i).g = 255;
            c->at(i).b = 0;
        } else if (ids[i] == 2) {
            c->at(i).r = 0;
            c->at(i).g = 0;
            c->at(i).b = 255;
        } else if (ids[i] == 3) {
            c->at(i).r = 255;
            c->at(i).g = 0;
            c->at(i).b = 255;
        } else if (ids[i] == 4) {
            c->at(i).r = 0;
            c->at(i).g = 255;
            c->at(i).b = 255;
        } else if (ids[i] == 5) {
            c->at(i).r = 255;
            c->at(i).g = 255;
            c->at(i).b = 0;
        } else {
            c->at(i).r = 127;
            c->at(i).g = 127;
            c->at(i).b = 127;
        }
    }
    visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb(c);
    viewer.addPointCloud<PointXYZRGB>(c, rgb, name);
    name[4]++;
}
int main(int argc, char** argv) {
    if (argc < 3) {
        cout << "Usage: globalreg cloud1 cloud2" << endl;
        return 0;
    }
    PointCloud<PointXYZ>::Ptr cloud1(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr cloud2(new PointCloud<PointXYZ>);
    cout << "Loading..." << endl;
    io::loadPCDFile<PointXYZ>(argv[1], *cloud1);
    io::loadPCDFile<PointXYZ>(argv[2], *cloud2);
    cout << "Done loading" << endl;

    vector<Vector4d> srcplanes, tgtplanes;
    vector<int> srcids, tgtids;
    //srcids.resize(cloud1->size(),-1);
    //tgtids.resize(cloud2->size(),-1);

    cout << "Finding planes..." << endl;
    findPlanes(cloud1, srcplanes, srcids);
    cout << "Done finding frame 1 planes" << endl;
    findPlanes(cloud2, tgtplanes, tgtids);
    cout << "Done finding planes" << endl;

    visualization::PCLVisualizer viewer("Cloud viewer");
    showColoredCloud(viewer, cloud1, srcids);

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }

    return 0;
}
