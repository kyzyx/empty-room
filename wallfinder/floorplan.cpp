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
                "     -ccw: Faces in counterclockwise direction (flip normals)\n" \
                "     -visualize: Show visualization (default on)\n" \
                "     -print_wall: Print out wall segments (default off)\n" \
                "     -all_candidates: show all potential wall segments (default off)\n" \
                "     -novisualize: Exit immediately\n" \
                "     -visualize_grid: show histogram of points instead of 3d model\n" \
                "     -wallthreshold int: number of points in a bin to be considered a possible wall\n" \
                "     -anglethreshold float: Angle between normals to be considered equal (default PI/40)\n" \
                "     -min_wall_length float: Minimum length of a wall (default 0.2)\n" \
                "     -resolution float: maximum distance for a point to be considered on a plane (default 0.01)\n" \
                );
        return 0;
    }
    // Parse arguments
    // OrientationFinder parameters
    // Display or not display
    bool ccw = false;
    bool visualize = true;
    bool print_wall = false;
    bool all_candidates = false;
    double anglethreshold = M_PI/40;
    double resolution = 0.01;
    double orientationresolution = 0.01;
    double minlength = 0.2;
    bool show_grid = false;
    int wallthreshold = 200;
    if (console::find_switch(argc, argv, "-ccw")) ccw = true;
    if (console::find_switch(argc, argv, "-visualize")) visualize = true;
    if (console::find_switch(argc, argv, "-print_wall")) print_wall = true;
    if (console::find_switch(argc, argv, "-novisualize")) visualize = false;
    if (console::find_switch(argc, argv, "-all_candidates")) all_candidates = true;
    if (console::find_switch(argc, argv, "-visualize_grid")) show_grid = true;
    if (console::find_argument(argc, argv, "-anglethreshold")) {
        console::parse_argument(argc, argv, "-anglethreshold", anglethreshold);
    }
    if (console::find_argument(argc, argv, "-resolution")) {
        console::parse_argument(argc, argv, "-resolution", resolution);
    }
    if (console::find_argument(argc, argv, "-orientationresolution")) {
        console::parse_argument(argc, argv, "-orientationresolution", orientationresolution);
    }
    if (console::find_argument(argc, argv, "-wallthreshold")) {
        console::parse_argument(argc, argv, "-wallthreshold", wallthreshold);
    }
    if (console::find_argument(argc, argv, "-min_wall_length")) {
        console::parse_argument(argc, argv, "-min_wall_length", minlength);
    }

    PolygonMesh::Ptr mesh(new PolygonMesh());
    io::loadPolygonFile(argv[1], *mesh);

    PlaneOrientationFinder of(mesh,orientationresolution);
    of.computeNormals(ccw);
    if (!of.computeOrientation()) {
        cout << "Error computing orientation! Non-triangle mesh!" << endl;
    }
    of.normalize();

    for (int i = 0; i < 3; ++i) {
        //cout << of.getAxis(i) << endl;
        cout << 180*acos(of.getAxis(i).dot(of.getAxis((i+1)%3)))/M_PI - 90 << " degrees from perpendicular" << endl;
    }
    WallFinder wf(&of, resolution);
    vector<int> labels(of.getCloud()->size(), WallFinder::LABEL_NONE);
    wf.findFloorAndCeiling(labels, anglethreshold);
    wf.findWalls(labels, wallthreshold, minlength, anglethreshold, all_candidates);
    if (print_wall) {
        for (int i = 0; i < wf.wallsegments.size(); ++i) {
            pair<int,int> x = wf.wallsegments[i].getCoords(wf.wallsegments[i].start);
            pair<int,int> y = wf.wallsegments[i].getCoords(wf.wallsegments[i].end);
            cout << "(" << x.first << "," << x.second << ") - ";
            cout << "(" << y.first << "," << y.second << ")";
            cout << endl;
        }
    }

    if (visualize) {
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
        if (show_grid) {
            PointCloud<PointXYZ>::Ptr grid(wf.getHistogram());
            PointCloud<PointXYZRGB>::Ptr colorgrid(new PointCloud<PointXYZRGB>());
            colorgrid->resize(grid->size());
            for (int i = 0; i < grid->size(); ++i) {
                colorgrid->at(i).x = grid->at(i).x;
                colorgrid->at(i).y = grid->at(i).y;
                colorgrid->at(i).z = grid->at(i).z;
                double count = grid->at(i).y;
                count = count*count*100;
                colorgrid->at(i).r = count > wallthreshold?255:0;
                colorgrid->at(i).g = colorgrid->at(i).y/2*255;
                colorgrid->at(i).b = 255-colorgrid->at(i).y/2*255;

            }

            visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb(colorgrid);
            viewer.addPointCloud<PointXYZRGB>(colorgrid, rgb, "Grid");

            //ModelCoefficients mc;
            //mc.values.resize(4);
            //mc.values[0] = 0;
            //mc.values[1] = 1;
            //mc.values[2] = 0;
            //mc.values[3] = -sqrt(2);
            //viewer.addPlane(mc);
        } else {
            viewer.addPointCloud<PointNormal>(walls, "Walls");
            //viewer.addPointCloudNormals<PointNormal>(walls, 20);
            viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 0,1,0,"Walls");
            viewer.addPointCloud<PointNormal>(floors, "Floors");
            //viewer.addPointCloudNormals<PointNormal>(floors, 20);
            viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 0,0,1,"Floors");
            viewer.addPointCloud<PointNormal>(ceils, "Ceilings");
            //viewer.addPointCloudNormals<PointNormal>(ceils, 20);
            viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 1,0,0,"Ceilings");
            viewer.addPointCloud<PointNormal>(corners, "Corners");
            //viewer.addPointCloudNormals<PointNormal>(ceils, 20);
            viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 1,0,1,"Corners");
            viewer.addPointCloud<PointNormal>(other, "Other");
            //viewer.addPointCloudNormals<PointNormal>(other, 20);
            viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 1,1,1,"Other");
        }

        viewer.initCameraParameters();
        viewer.setCameraPosition(-1,-1,-1,0,0,1,0,-1,0);
        viewer.addCoordinateSystem();
        PointXYZ yax(of.getAxis(0)(0), of.getAxis(0)(1), of.getAxis(0)(2));
        PointXYZ xax(of.getAxis(1)(0), of.getAxis(1)(1), of.getAxis(1)(2));
        PointXYZ zax(of.getAxis(2)(0), of.getAxis(2)(1), of.getAxis(2)(2));
        char n[] = {'L', 'i', '0'};
        for (int i = 0; i < wf.wallsegments.size(); ++i) {
            if (wf.wallsegments[i].direction == 0) {
                PointXYZ start(wf.wallsegments[i].coord, 0, wf.wallsegments[i].start);
                PointXYZ end(wf.wallsegments[i].coord, 0, wf.wallsegments[i].end);
                viewer.addLine(start, end, 1, 0, i/(double)wf.wallsegments.size(), n);
                n[2]++;
            } else {
                PointXYZ start(wf.wallsegments[i].start, 0, wf.wallsegments[i].coord);
                PointXYZ end(wf.wallsegments[i].end, 0, wf.wallsegments[i].coord);
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
    }

    return 0;
}
