#include "pairwise.h"
#include "findplanes.h"
#include <pcl/io/io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl;
using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
    visualization::PCLVisualizer viewer("Cloud viewer");

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }

    return 0;
}
