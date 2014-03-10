#include <pcl/io/io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/console/parse.h>

#include <boost/thread/thread.hpp>

#include <iostream>

#include "colorhelper.h"


using namespace std;
using namespace pcl;

int main(int argc, char* argv[]) {
    if (argc < 5) {
        printf("Usage: invrender mesh.ply imagelist.txt lightimagelist.txt camera.cam [args]\n" \
                "   Arguments:\n" \
                );
        return 0;
    }
    // Parse arguments
    /*if (console::find_switch(argc, argv, "-ccw")) ccw = true;
    if (console::find_argument(argc, argv, "-anglethreshold")) {
        console::parse_argument(argc, argv, "-anglethreshold", anglethreshold);
    }*/

    PolygonMesh::Ptr mesh(new PolygonMesh());
    io::loadPolygonFile(argv[1], *mesh);
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
    fromPCLPointCloud2(mesh->cloud, *cloud);

    ColorHelper loader;
    loader.load(argv[2], argv[4]);
    cout << loader.size() << endl;

    visualization::PCLVisualizer viewer("Cloud viewer");
    viewer.addPointCloud<PointXYZ>(cloud, "Mesh");
    char n[] = {'A', '0'};
    for (int i = 0; i < loader.size(); i++) {
        R3Point p = loader.getCamera(i)->pos;
        PointXYZ p1(loader.getCamera(i)->pos[0], loader.getCamera(i)->pos[1], loader.getCamera(i)->pos[2]);
        R3Point v2 = p + loader.getCamera(i)->towards*0.2;
        R3Point v3 = p + loader.getCamera(i)->up*0.2;
        PointXYZ p2(v2[0], v2[1], v2[2]);
        PointXYZ p3(v3[0], v3[1], v3[2]);
        n[0] = 'A';
        viewer.addLine(p2, p1, 1, 0, 0, n);
        n[0] = 'B';
        viewer.addLine(p3, p1, 0, 0, 1, n);
        n[1]++;
    }
    // Mesh Colorer - Stores colors and rays for each face in mesh, labels lights
    // For each pixel in each image, add sample to mesh colorer
    // Label lights from light images
    // CHECK - display colored and uncolored faces (antialiasing a problem?)
    // Inverse render

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::seconds(0.5));
    }
    return 0;
}
