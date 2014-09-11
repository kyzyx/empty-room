#include <GL/glew.h>
#include <GL/glut.h>
#include <pcl/io/io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <boost/thread/thread.hpp>
#include <iostream>

#include "colorhelper.h"
#include "mesh.h"
#include "reproject.h"
#include "rerender.h"
#include "orientation_finder.h"
#include "wall_finder.h"
#include "clusterlights.h"

#include "parse_args.h"
#include "display.h"
#include "solver.h"


using namespace std;
using namespace pcl;

int main(int argc, char* argv[]) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(1,1);
    glutCreateWindow("");
    glutHideWindow();
    GLenum err = glewInit();
    if (err != GLEW_OK) {
        cerr << "Error:" << glewGetErrorString(err) << endl;
        return 1;
    }
    if (!parseargs(argc, argv)) return 1;
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

    vector<int> wallindices;
    vector<int> labels(cloud->size(), WallFinder::LABEL_NONE);
    WallFinder wf(resolution);
    if (wallinput) {
        cout << "Loading wall files..." << endl;
        wf.loadWalls(wallfile, labels);
        for (int i = 0; i < labels.size(); ++i) {
            if (labels[i] == WallFinder::LABEL_WALL) wallindices.push_back(i);
        }
    } else if (do_wallfinding) {
        PlaneOrientationFinder of(mesh,0.01);
        of.computeNormals(ccw);
        if (!of.computeOrientation()) {
            cout << "Error computing orientation! Non-triangle mesh!" << endl;
        }
        cout << "Done analyzing geometry" << endl;
        // FIXME: Transform camera positions from normalize
        //of.normalize();
        wf.findFloorAndCeiling(of, labels, anglethreshold);
        wf.findWalls(of, labels, wallthreshold, minlength, anglethreshold);
        // FIXME: if (output_wall) ;
        cout << "Done finding walls" << endl;
        for (int i = 0; i < labels.size(); ++i) {
            if (labels[i] == WallFinder::LABEL_WALL) wallindices.push_back(i);
        }
        if (output_wall) wf.saveWalls(walloutfile, labels);
    }

    ColorHelper loader;
    Mesh m(mesh,true);

    if (input) {
        cout << "Loading reprojection input files..." << endl;
        loader.readMayaCameraFile(camfile);
        m.readSamples(infile);
        cout << "Done loading samples" << endl;
    } else if (do_reprojection) {
        cout << "Reading input files..." << endl;
        loader.load(imagelist, camfile);
        cout << "Done reading color images " << loader.size() << endl;
        if (hdr) {
            if (all_project) {
                reproject(loader, m, hdr_threshold);
            } else {
                //const float* hdrimg = (const float*) loader.getImage(project);
                reproject((float*) loader.getImage(project), loader.getCamera(project), m, hdr_threshold);
            }
        } else {
            ColorHelper lights;
            lights.load(lightimagelist, camfile);
            cout << "Done reading lighting images " << lights.size() << endl;


            if (all_project) {
                reproject(loader, lights, m);
            } else {
                reproject(loader.getImage(project), lights.getImage(project), loader.getCamera(project), m);
            }
        }
        cout << "Done reprojecting" << endl;
        int numlights = clusterLights(m);
        cout << "Done clustering " << numlights << " lights" << endl;
        if (output_reprojection) m.writeSamples(outfile);
    }
    m.computeColorsOGL();

    InverseRender ir(&m);
    // Only do inverse rendering with full reprojection and wall labels
    if ((input || all_project) && (wallinput || do_wallfinding)) {
        if (do_sampling) {
            if (read_eq) {
                ir.loadVariablesBinary(samplefile);
            } else {
                ir.calculate(wallindices, numsamples, discardthreshold, 2);
                if (write_eq) {
                    ir.writeVariablesMatlab("eq.m");
                    ir.writeVariablesBinary(sampleoutfile);
                }
            }
            ir.solve();
        }
        outputRadianceFile(radfile, wf, m, ir);
    }

    if (display) {
        int labeltype = LABEL_LIGHTS;
        if (project_debug) labeltype = LABEL_REPROJECT_DEBUG;
        visualize(m, cloud, loader, ir, wf, labeltype, camera);
    }
    return 0;
}
