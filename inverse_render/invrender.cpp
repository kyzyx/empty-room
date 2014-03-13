#include <pcl/io/io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <iostream>

#include "colorhelper.h"
#include "mesh.h"
#include "reproject.h"
#include "orientation_finder.h"
#include "wall_finder.h"

#include "parse_args.h"
#include "display.h"


using namespace std;
using namespace pcl;

int main(int argc, char* argv[]) {
    parseargs(argc, argv);
    PolygonMesh::Ptr mesh(new PolygonMesh());
    io::loadPolygonFile(argv[1], *mesh);
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>());
    {
        PointCloud<PointXYZ> tmp;
        fromPCLPointCloud2(mesh->cloud, tmp);
        cloud->points.resize(tmp.size());
        for (size_t i = 0; i < tmp.size(); ++i) {
            cloud->points[i].x = tmp[i].x;
            cloud->points[i].y = tmp[i].y;
            cloud->points[i].z = tmp[i].z;
        }
    }
    cout << "Done loading mesh geometry" << endl;

    if (do_wallfinding) {
        PlaneOrientationFinder of(mesh,0.01);
        of.computeNormals(ccw);
        if (!of.computeOrientation()) {
            cout << "Error computing orientation! Non-triangle mesh!" << endl;
        }
        cout << "Done analyzing geometry" << endl;
        // FIXME: Transform camera positions from normalize
        //of.normalize();
        WallFinder wf;
        vector<int> labels(of.getCloud()->size(), WallFinder::LABEL_NONE);
        wf.findFloorAndCeiling(of, labels, resolution, anglethreshold);
        wf.findWalls(of, labels, minlength, resolution, anglethreshold);
        // FIXME: if (output_wall) ;
        cout << "Done finding walls" << endl;
    }

    ColorHelper loader;
    Mesh m(mesh);

    if (input) {
        cout << "Loading input files..." << endl;
        loader.readMayaCameraFile(camfile);
        m.readSamples(infile);
        cout << "Done loading samples" << endl;
    } else if (do_reprojection) {
        cout << "Reading input files..." << endl;
        loader.load(imagelist, camfile);
        cout << "Done reading color images " << loader.size() << endl;
        ColorHelper lights;
        lights.load(lightimagelist, camfile);
        cout << "Done reading lighting images " << lights.size() << endl;

        if (all_project) {
            reproject(loader, lights, m);
        } else {
            reproject(loader.getImage(project), lights.getImage(project), loader.getCamera(project), m);
        }
        cout << "Done reprojecting" << endl;
        if (output_reprojection) m.writeSamples(outfile);
    }

    if (display) {
        visualize(m, cloud, loader, show_frustrum, prune, all_cameras, project_debug, camera);
    }
    return 0;
}
