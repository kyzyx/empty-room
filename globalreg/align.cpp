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

    PointCloud<PointXYZRGB>::Ptr colored1(new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr colored2(new PointCloud<PointXYZRGB>);

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }
    return 0;
}
