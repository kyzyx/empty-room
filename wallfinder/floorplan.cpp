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
    of.normalize();

    for (int i = 0; i < 3; ++i) {
        //cout << of.getAxis(i) << endl;
        cout << 180*acos(of.getAxis(i).dot(of.getAxis((i+1)%3)))/M_PI - 90 << " degrees from perpendicular" << endl;
    }
    WallFinder wf;
    vector<int> labels(of.getCloud()->size(), WallFinder::LABEL_NONE);
    wf.findFloorAndCeiling(of, labels);
    wf.findWalls(of, labels);

    vector<int> wallindices, floorindices, ceilindices, cornerindices, indices;
    for (int i = 0; i < labels.size(); ++i) {
        switch(labels[i]) {
            case WallFinder::LABEL_WALL: wallindices.push_back(i);
                             break;
            case WallFinder::LABEL_FLOOR: floorindices.push_back(i);
                             break;
            case WallFinder::LABEL_CEILING: ceilindices.push_back(i);
                             break;
            case WallFinder::LABEL_CORNER: cornerindices.push_back(i);
                             break;
            default: indices.push_back(i);
        }
    }

    PointCloud<PointNormal>::Ptr walls(new PointCloud<PointNormal>(*(of.getCloud()), wallindices));
    PointCloud<PointNormal>::Ptr floors(new PointCloud<PointNormal>(*(of.getCloud()), floorindices));
    PointCloud<PointNormal>::Ptr ceils(new PointCloud<PointNormal>(*(of.getCloud()), ceilindices));
    PointCloud<PointNormal>::Ptr corners(new PointCloud<PointNormal>(*(of.getCloud()), cornerindices));
    PointCloud<PointNormal>::Ptr other(new PointCloud<PointNormal>(*(of.getCloud()), indices));

    visualization::PCLVisualizer viewer("Room");
    viewer.setBackgroundColor(0,0,0);

    viewer.addPointCloud<PointNormal>(walls, "Walls");
    //viewer.addPointCloudNormals<PointNormal>(walls, 20);
    viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 0,1,0,"Walls");
    viewer.addPointCloud<PointNormal>(floors, "Floors");
    //viewer.addPointCloudNormals<PointNormal>(floors, 20);
    viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 1,0,0,"Floors");
    viewer.addPointCloud<PointNormal>(ceils, "Ceilings");
    //viewer.addPointCloudNormals<PointNormal>(ceils, 20);
    viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 0,0,1,"Ceilings");
    viewer.addPointCloud<PointNormal>(corners, "Corners");
    //viewer.addPointCloudNormals<PointNormal>(ceils, 20);
    viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 1,0,1,"Corners");
    viewer.addPointCloud<PointNormal>(other, "Other");
    //viewer.addPointCloudNormals<PointNormal>(other, 20);
    viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 1,1,1,"Other");

    viewer.initCameraParameters();
    viewer.setCameraPosition(-1,-1,-1,0,0,1,0,-1,0);
    viewer.addCoordinateSystem();
    PointXYZ yax(of.getAxis(0)(0), of.getAxis(0)(1), of.getAxis(0)(2));
    PointXYZ xax(of.getAxis(1)(0), of.getAxis(1)(1), of.getAxis(1)(2));
    PointXYZ zax(of.getAxis(2)(0), of.getAxis(2)(1), of.getAxis(2)(2));
    char n[] = {'L', 'i', '0'};
    double res = 0.01;
    for (int i = 0; i < wf.wallsegments.size(); ++i) {
        if (wf.wallsegments[i].direction == 0) {
            PointXYZ start(wf.wallsegments[i].coord*res, 0, wf.wallsegments[i].start*res);
            PointXYZ end(wf.wallsegments[i].coord*res, 0, wf.wallsegments[i].end*res);
            viewer.addLine(start, end, 1, 0, i/(double)wf.wallsegments.size(), n);
            n[2]++;
        } else {
            PointXYZ start(wf.wallsegments[i].start*res, 0, wf.wallsegments[i].coord*res);
            PointXYZ end(wf.wallsegments[i].end*res, 0, wf.wallsegments[i].coord*res);
            viewer.addLine(start, end, 1, 0, i/(double)wf.wallsegments.size(), n);
            n[2]++;
        }
    }
    viewer.addArrow(yax, PointXYZ(0,0,0), 0., 1., 0., false, "yaxis", 0);
    viewer.addArrow(xax, PointXYZ(0,0,0), 1., 0., 0., false, "xaxis", 0);
    viewer.addArrow(zax, PointXYZ(0,0,0), 0., 0., 1., false, "zaxis", 0);
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::seconds(0.5));
    }

    return 0;
}
