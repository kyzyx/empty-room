#include <pcl/io/io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

#include <boost/thread/thread.hpp>

#include <iostream>

#include "orientation_finder.h"
#include "wall_finder.h"


using namespace std;
using namespace pcl;

int main(int argc, char* argv[]) {
    if (argc < 3) {
        printf("Usage: wallfinder filename.ply output.txt [args]\n" \
                "   Arguments:\n" \
                );
        return 0;
    }
    // Parse arguments
    // OrientationFinder parameters
    // Display or not display
    bool ccw = false;
    if (console::find_switch(argc, argv, "-ccw")) ccw = true;
    /*bool rotate = true;
    double noise = 0.005;
    if (console::find_switch(argc, argv, "-rotate")) rotate = true;
    if (console::find_switch(argc, argv, "-norotate")) rotate = false;
    if (console::find_argument(argc, argv, "-noise")) {
        console::parse_argument(argc, argv, "-noise", noise);
    }*/

    PolygonMesh::Ptr mesh(new PolygonMesh());
    io::loadPolygonFile(argv[1], *mesh);

    OrientationFinder of(mesh);
    of.computeNormals(ccw);
    if (!of.computeOrientation()) {
        cout << "Error computing orientation! Non-triangle mesh!" << endl;
    }

    for (int i = 0; i < 3; ++i) {
        cout << of.getAxis(i) << endl;
        cout << 180*acos(of.getAxis(i).dot(of.getAxis((i+1)%3)))/M_PI - 90 << " degrees from perpendicular" << endl;
    }
    WallFinder wf;
    vector<int> labels(of.getCloud()->size());
    wf.findFloorAndCeiling(of, labels);

    visualization::PCLVisualizer viewer("Room");
    viewer.setBackgroundColor(0,0,0);
    viewer.addPointCloud<PointNormal>(of.getCloud(), "Room");
    viewer.addPointCloudNormals<PointNormal>(of.getCloud(), 20);
    viewer.initCameraParameters();
    viewer.setCameraPosition(0,0,0,0,0,1,0,-1,0);
    PointXYZ yax(of.getAxis(0)(0), of.getAxis(0)(1), of.getAxis(0)(2));
    PointXYZ xax(of.getAxis(1)(0), of.getAxis(1)(1), of.getAxis(1)(2));
    PointXYZ zax(of.getAxis(2)(0), of.getAxis(2)(1), of.getAxis(2)(2));
    viewer.addArrow(yax, PointXYZ(0,0,0), 0., 1., 0., false, "yaxis", 0);
    viewer.addArrow(xax, PointXYZ(0,0,0), 1., 0., 0., false, "xaxis", 0);
    viewer.addArrow(zax, PointXYZ(0,0,0), 0., 0., 1., false, "zaxis", 0);
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::seconds(0.5));
    }

    return 0;
}
