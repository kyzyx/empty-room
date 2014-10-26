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
    vector<int> floorindices;
    PlaneOrientationFinder of(mesh, 0.01);
    of.computeNormals(ccw);
    if (!of.computeOrientation()) {
        cout << "Error computing orientation! Non-triangle mesh!" << endl;
    }
    cout << "Done analyzing geometry" << endl;
    of.normalize();

    ColorHelper loader;
    Mesh m(mesh,true);
    WallFinder wf(&of, resolution);

    if (wallinput) {
        cout << "Loading wall files..." << endl;
        wf.loadWalls(wallfile, m.types);
        for (int i = 0; i < m.types.size(); ++i) {
            if (m.types[i] == WallFinder::LABEL_WALL) wallindices.push_back(i);
            else if (m.types[i] == WallFinder::LABEL_FLOOR) floorindices.push_back(i);
        }
    } else if (do_wallfinding) {
        wf.findFloorAndCeiling(m.types, anglethreshold);
        wf.findWalls(m.types, wallthreshold, minlength, anglethreshold);
        cout << "Done finding walls" << endl;
        for (int i = 0; i < m.types.size(); ++i) {
            if (m.types[i] == WallFinder::LABEL_WALL) wallindices.push_back(i);
            else if (m.types[i] == WallFinder::LABEL_FLOOR) floorindices.push_back(i);
        }
        if (output_wall) wf.saveWalls(walloutfile, m.types);
    }

    int numlights;
    if (input) {
        cout << "Loading reprojection input files..." << endl;
        loader.readCameraFile(camfile);
        numlights = m.readSamples(infile);
        cout << "Done loading samples" << endl;
    } else if (do_reprojection) {
        cout << "Reading input files..." << endl;
        loader.load(camfile);
        cout << "Done reading color images " << loader.size() << endl;
        if (all_project) {
            reproject(loader, m, hdr_threshold, image_flip_x, image_flip_y);
        } else {
            reproject((float*) loader.getImage(project), loader.getCamera(project), m, hdr_threshold, image_flip_x, image_flip_y);
        }
        cout << "Done reprojecting" << endl;
        numlights = clusterLights(m);
        cout << "Done clustering " << numlights << " lights" << endl;
        if (output_reprojection) m.writeSamples(outfile);
    }
    m.computeColorsOGL();

    if (do_reprojection) {
        hemicuberesolution = max(hemicuberesolution, loader.getCamera(0)->width);
        hemicuberesolution = max(hemicuberesolution, loader.getCamera(0)->height);
    }
    InverseRender ir(&m, numlights, hemicuberesolution);
    vector<SampleData> walldata, floordata;
    // Only do inverse rendering with full reprojection and wall labels
    if ((input || all_project) && (wallinput || do_wallfinding)) {
        if (do_sampling) {
            if (read_eq) {
                ir.loadVariablesBinary(walldata, samplefile + ".walls");
                ir.loadVariablesBinary(floordata, samplefile + ".floors");
            } else {
                ir.computeSamples(walldata, wallindices, numsamples, discardthreshold);
                ir.computeSamples(floordata, floorindices, numsamples, discardthreshold);
                if (write_eq) {
                    ir.writeVariablesBinary(walldata, sampleoutfile + ".walls");
                    ir.writeVariablesBinary(floordata, sampleoutfile + ".floors");
                }
                if (write_matlab) {
                    ir.writeVariablesMatlab(walldata, matlabsamplefile);
                }
            }
            ir.solve(walldata);
            Eigen::Vector3f floornormal(0,1,0);
            floornormal = of.getNormalizationTransform().inverse().topLeftCorner(3,3)*floornormal;
            Texture tex;
            loader.load(camfile);
            ir.solveTexture(floordata, &loader, floornormal, tex);
            cout << "Done solving texture..." << endl;
            if (tex.size > 0) {
                ColorHelper::writeExrImage("texture.exr", tex.texture, tex.size, tex.size);
            }
            if (radfile != "") {
                outputRadianceFile(radfile, wf, m, ir);
            }
        }
    }

    if (display) {
        int labeltype = LABEL_LIGHTS;
        if (project_debug) labeltype = LABEL_REPROJECT_DEBUG;
        visualize(cloud, loader, ir, wf, labeltype, camera, walldata);
    }
    return 0;
}
