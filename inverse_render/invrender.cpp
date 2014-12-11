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

inline R3Vector eigen2gaps(Eigen::Vector3f v) {
    return R3Vector(v(0), v(1), v(2));
}

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
    cout << "Loading mesh geometry...." << endl;
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
    PlaneOrientationFinder of(mesh, resolution/2);
    of.computeNormals(ccw);
    Mesh m(mesh,true);
    cout << "Done loading mesh geometry" << endl;

    vector<int> wallindices;
    vector<int> floorindices;
    ColorHelper loader(image_flip_x, image_flip_y);
    WallFinder wf(&of, resolution);

    if (do_wallfinding) {
        cout << "===== WALLFINDING =====" << endl;
        if (wallinput) {
            cout << "Loading wall files..." << endl;
            wf.loadWalls(wallfile, m.types);
            cout << "Done loading wall files..." << endl;
            of.normalize();
        } else {
            cout << "Analyzing geometry..." << endl;
            if (!of.computeOrientation()) {
                cout << "Error computing orientation! Non-triangle mesh!" << endl;
            }
            cout << "Done analyzing geometry" << endl;
            of.normalize();
            cout << "Finding walls..." << endl;
            wf.findFloorAndCeiling(m.types, anglethreshold);
            wf.findWalls(m.types, wallthreshold, minlength, anglethreshold);
            cout << "Done finding walls" << endl;
        }
        if (flipfloorceiling) {
            for (int i = 0; i < m.types.size(); ++i) {
                if (m.types[i] == WallFinder::LABEL_CEILING) m.types[i] = WallFinder::LABEL_FLOOR;
                else if (m.types[i] == WallFinder::LABEL_FLOOR) m.types[i] = WallFinder::LABEL_CEILING;
            }
        }
        for (int i = 0; i < m.types.size(); ++i) {
            if (m.types[i] == WallFinder::LABEL_WALL) wallindices.push_back(i);
            else if (m.types[i] == WallFinder::LABEL_FLOOR) floorindices.push_back(i);
        }
        if (output_wall) wf.saveWalls(walloutfile, m.types);
        cout << "=======================" << endl;
    }

    int numlights = 0;
    if (do_reprojection) {
        cout << "===== REPROJECTING =====" << endl;
        if (input) {
            cout << "Loading reprojection input files..." << endl;
            loader.readCameraFile(camfile);
            cout << "Reading input files..." << endl;
            loader.load(camfile, ColorHelper::READ_COLOR);
            if (use_confidence_files) {
                loader.load(camfile, ColorHelper::READ_CONFIDENCE);
            }
            cout << "Done reading " << loader.size() << " color images" << endl;
            if (do_wallfinding) {
                vector<char> tmp;
                swap(tmp, m.types);
                numlights = m.readSamples(infile);
                swap (m.types, tmp);
            } else {
                numlights = m.readSamples(infile);
            }
            cout << "Done loading reprojection files" << endl;
            if (hdr_threshold > 0) {
                numlights = clusterLights(m, hdr_threshold, minlightsize);
                cout << "Done clustering " << numlights << " lights" << endl;
            }
        } else {
            if (hdr_threshold < 0) hdr_threshold = 10.0;
            if (all_project) {
                cout << "Reading input files..." << endl;
                loader.load(camfile, ColorHelper::READ_COLOR | ColorHelper::READ_DEPTH);
                if (use_confidence_files) {
                    loader.load(camfile, ColorHelper::READ_CONFIDENCE);
                }
                cout << "Done reading " << loader.size() << " color images" << endl;
                cout << "Reprojecting..." << endl;
                reproject(loader, m, hdr_threshold);
            } else {
                loader.load(camfile, 0);
                loader.load(project, ColorHelper::READ_COLOR | ColorHelper::READ_DEPTH);
                if (use_confidence_files) {
                    loader.load(project, ColorHelper::READ_CONFIDENCE);
                }
                cout << "Reprojecting..." << endl;
                reproject((const float*) loader.getImage(project),
                          loader.getConfidenceMap(project),
                          loader.getDepthMap(project),
                          loader.getCamera(project),
                          m, hdr_threshold);
            }
            cout << "Done reprojecting; clustering lights..." << endl;
            numlights = clusterLights(m, hdr_threshold, minlightsize);
            cout << "Done clustering " << numlights << " lights" << endl;
        }
        if (output_reprojection) m.writeSamples(outfile);
        if (coloredfile.length()) m.writeColoredMesh(coloredfile, displayscale);
        m.computeColorsOGL();
        hemicuberesolution = max(hemicuberesolution, loader.getCamera(0)->width);
        hemicuberesolution = max(hemicuberesolution, loader.getCamera(0)->height);
        cout << "========================" << endl;
    }
    InverseRender ir(&m, numlights, hemicuberesolution);
    vector<SampleData> walldata, floordata;
    // Only do inverse rendering with full reprojection and wall labels
    if (do_sampling && (input || all_project) && (wallinput || do_wallfinding)) {
        cout << "==== SAMPLING SCENE ====" << endl;
        if (read_eq) {
            ir.loadVariablesBinary(walldata, samplefile + ".walls");
            if (do_texture) {
                ir.loadVariablesBinary(floordata, samplefile + ".floors");
            }
        } else {
            ir.computeSamples(walldata, wallindices, numsamples, discardthreshold);
            if (do_texture) {
                ir.computeSamples(floordata, floorindices, numsamples, discardthreshold);
            }
            if (write_eq) {
                ir.writeVariablesBinary(walldata, sampleoutfile + ".walls");
                if (do_texture) {
                    ir.writeVariablesBinary(floordata, sampleoutfile + ".floors");
                }
            }
            if (write_matlab) {
                ir.writeVariablesMatlab(walldata, matlabsamplefile);
            }
        }
        cout << "========================" << endl;
        ir.setNumRansacIters(numRansacIters);
        ir.setMaxPercentErr(maxPercentErr);
        ir.solve(walldata);

        Texture tex;
        if (do_texture) {
            // Prepare floor plane
            Eigen::Matrix4f t = of.getNormalizationTransform().inverse();
            Eigen::Vector3f floornormal(0,flipfloorceiling?-1:1,0);
            floornormal = t.topLeftCorner(3,3)*floornormal;
            Eigen::Vector4f floorpoint(0, wf.floorplane, 0, 1);
            floorpoint = t*floorpoint;
            R3Plane floorplane(eigen2gaps(floorpoint.head(3)).Point(), eigen2gaps(floornormal));

            loader.load(camfile, use_confidence_files);

            cout << "Solving texture..." << endl;
            ir.solveTexture(floordata, &loader, floorplane, tex);
            cout << "Done solving texture..." << endl;
            if (tex.size > 0) {
                ColorHelper::writeExrImage(texfile, tex.texture, tex.size, tex.size);
            }
        }
        if (radfile != "") {
            outputRadianceFile(radfile, wf, m, ir);
        }
        if (plyfile != "") {
            outputPlyFile(plyfile, wf, m, ir);
        }
        if (pbrtfile != "") {
            outputPbrtFile(pbrtfile, wf, m, ir, tex, loader.getCamera(0), do_texture?texfile:"");
        }
    }
    cout << "DONE PROCESSING" << endl;

    if (display) {
        InvRenderVisualizer irv(cloud, loader, wf, ir);
        if (project_debug) irv.recalculateColors(InvRenderVisualizer::LABEL_REPROJECT_DEBUG);
        irv.visualizeCameras(camera);
        irv.visualizeWalls();
        irv.addSamples(walldata);
        irv.loop();
    }
    return 0;
}
