#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/vtk_lib_io.h>

#include <cstdio>
#include <cstdlib>
#include <random>

using namespace std;
using namespace pcl;
const double PI = 3.14165926536;

int main(int argc, char* argv[]) {
    if (argc < 3) {
        printf("Usage: synthetic_preprocess filename.ply output.ply [args]\n" \
                "   Arguments:\n" \
                "   -noise float: max noise to add to each coordinate in each point (default 0.001)\n" \
                "   -rotate: rotate randomly some amount (default on)\n" \
                "   -norotate: keep same orientation\n" \
                );
        return 0;
    }
    // Parse arguments
    bool rotate = true;
    double noise = 0.005;
    if (console::find_switch(argc, argv, "-rotate")) rotate = true;
    if (console::find_switch(argc, argv, "-norotate")) rotate = false;
    if (console::find_argument(argc, argv, "-noise")) {
        console::parse_argument(argc, argv, "-noise", noise);
    }

    PolygonMesh mesh;
    io::loadPolygonFile(argv[1], mesh);

    PointCloud<PointXYZ> cloud;
    fromPCLPointCloud2(mesh.cloud, cloud);

    // Generate random generator
    default_random_engine gen;
    gen.seed(time(0));
    uniform_real_distribution<double> angle(0,1);
    uniform_real_distribution<double> dist(-noise,noise);
    // Generate global rotation, which keeps down approximately down
    double yaw = angle(gen)*2*PI;
    double pitch = angle(gen)*PI/24;
    double roll = angle(gen)*PI/24;
    Eigen::Matrix4f rot1, rot2, rot3;
    rot1 << cos(yaw), -sin(yaw), 0, 0,
            sin(yaw),  cos(yaw), 0, 0,
                   0,         0, 1, 0,
                   0,         0, 0, 1;
    rot2 << cos(pitch),         0, sin(pitch), 0,
                     0,         1,          0, 0,
           -sin(pitch),         0, cos(pitch), 0,
                     0,         0,          0, 1;
    rot3 <<          1,         0,          0, 0,
                     0, cos(roll), -sin(roll), 0,
                     0, sin(roll),  cos(roll), 0,
                     0,         0,          0, 1;
    printf("Yaw: %.4f, Pitch: %.4f, Roll: %.4f\n", yaw, pitch, roll);
    transformPointCloud(cloud, cloud, rot3*rot2*rot1);

    // Add noise to all points
    for (PointCloud<PointXYZ>::iterator it = cloud.begin(); it != cloud.end(); ++it) {
        (*it).x += dist(gen);
        (*it).y += dist(gen);
        (*it).z += dist(gen);
    }

    toPCLPointCloud2(cloud, mesh.cloud);

    io::savePolygonFile(argv[2], mesh);

    return 0;
}
