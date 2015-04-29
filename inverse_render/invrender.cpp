#include "opengl_compat.h"
#include <pcl/io/io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <iostream>

#include "meshserver.h"
#include "imageserver.h"
#include "imageio.h"
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
    if (!openglInit()) return 1;
    if (!parseargs(argc, argv)) return 1;

    cout << "Loading mesh geometry...." << endl;
    MeshServer mmgr(argv[1], ccw);
    cout << "Done loading mesh geometry" << endl;

    ImageServer imgr(camfile, image_flip_x, image_flip_y);
    // if (use_confidence_files)


    vector<int> wallindices;
    vector<int> floorindices;
    WallFinder wf(resolution);

    if (do_wallfinding) {
        cout << "===== WALLFINDING =====" << endl;
        vector<char> types(mmgr.NVertices());
        PlaneOrientationFinder of(&mmgr, resolution/2);
        of.computeNormals();
        if (wallinput) {
            cout << "Loading wall files..." << endl;
            wf.loadWalls(wallfile, types, of);
            cout << "Done loading wall files..." << endl;
            of.normalize();
            wf.setNormalizationTransform(of.getNormalizationTransform());
        } else {
            cout << "Analyzing geometry..." << endl;
            if (!of.computeOrientation()) {
                cout << "Error computing orientation! Non-triangle mesh!" << endl;
            }
            cout << "Done analyzing geometry" << endl;
            of.normalize();
            wf.setNormalizationTransform(of.getNormalizationTransform());
            cout << "Finding walls..." << endl;
            wf.findFloorAndCeiling(&of, types, anglethreshold);
            cout << "Done finding floor/ceiling" << endl;
            wf.findWalls(&of, types, wallthreshold, minlength, anglethreshold);
            cout << "Done finding walls" << endl;
        }
        if (flipfloorceiling) {
            for (int i = 0; i < types.size(); ++i) {
                if (types[i] == WallFinder::LABEL_CEILING) types[i] = WallFinder::LABEL_FLOOR;
                else if (types[i] == WallFinder::LABEL_FLOOR) types[i] = WallFinder::LABEL_CEILING;
            }
        }
        for (int i = 0; i < types.size(); ++i) {
            mmgr.setLabel(i, types[i], 1);
            if (types[i] == WallFinder::LABEL_WALL) wallindices.push_back(i);
            else if (types[i] == WallFinder::LABEL_FLOOR) floorindices.push_back(i);
        }
        if (output_wall) wf.saveWalls(walloutfile, types, of);
        cout << "=======================" << endl;
    }

    int numlights = 0;
    if (do_reprojection) {
        cout << "===== REPROJECTING =====" << endl;
        if (input) {
            mmgr.readSamplesFromFile(infile);
            cout << "Done loading reprojection files" << endl;
            if (hdr_threshold > 0) {
                numlights = clusterLights(mmgr, hdr_threshold, minlightsize);
                cout << "Done clustering " << numlights << " lights" << endl;
            }
        } else {
            if (hdr_threshold < 0) hdr_threshold = 10.0;
            if (all_project) {
                cout << "Reprojecting..." << endl;
                reproject(imgr, mmgr, hdr_threshold);
            } else {
                cout << "Reprojecting..." << endl;
                reproject((const float*) imgr.getImage(project),
                          (const float*) imgr.getImage("confidence", project),
                          (const float*) imgr.getImage("depth", project),
                          imgr.getCamera(project),
                          mmgr, hdr_threshold);
            }
            mmgr.commitSamples();
            cout << "Done reprojecting; clustering lights..." << endl;
            numlights = clusterLights(mmgr, hdr_threshold, minlightsize);
            cout << "Done clustering " << numlights << " lights" << endl;
        }
        //if (output_reprojection) FIXME.writeSamples(outfile);
        //if (coloredfile.length()) FIXME.writeColoredMesh(coloredfile, displayscale);
        hemicuberesolution = max(hemicuberesolution, imgr.getCamera(0)->width);
        hemicuberesolution = max(hemicuberesolution, imgr.getCamera(0)->height);
        cout << "========================" << endl;
    }
    InverseRender ir(&mmgr, numlights, hemicuberesolution);
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
            Eigen::Matrix4f t = wf.getNormalizationTransform().inverse();
            Eigen::Vector3f floornormal(0,flipfloorceiling?-1:1,0);
            floornormal = t.topLeftCorner(3,3)*floornormal;
            Eigen::Vector4f floorpoint(0, wf.floorplane, 0, 1);
            floorpoint = t*floorpoint;
            R3Plane floorplane(eigen2gaps(floorpoint.head(3)).Point(), eigen2gaps(floornormal));

            cout << "Solving texture..." << endl;
            ir.solveTexture(floordata, &imgr, floorplane, tex);
            cout << "Done solving texture..." << endl;
            if (tex.size > 0) {
                ImageIO::writeExrImage(texfile, tex.texture, tex.size, tex.size);
            }
        }
        if (radfile != "") {
            outputRadianceFile(radfile, wf, mmgr, ir);
        }
        if (plyfile != "") {
            outputPlyFile(plyfile, wf, mmgr, ir);
        }
        if (pbrtfile != "") {
            outputPbrtFile(pbrtfile, wf, mmgr, ir, tex, imgr.getCamera(0), do_texture?texfile:"");
        }
    }
    cout << "DONE PROCESSING" << endl;

    if (display) {
        PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>());
        {
            cloud->points.resize(mmgr.NVertices());
            for (size_t i = 0; i < mmgr.NVertices(); ++i) {
                cloud->points[i].x = mmgr.VertexPosition(i).X();
                cloud->points[i].y = mmgr.VertexPosition(i).Y();
                cloud->points[i].z = mmgr.VertexPosition(i).Z();
            }
        }

        InvRenderVisualizer irv(cloud, imgr, wf, ir);
        if (project_debug) irv.recalculateColors(InvRenderVisualizer::LABEL_REPROJECT_DEBUG);
        irv.visualizeCameras(camera);
        irv.visualizeWalls();
        irv.addSamples(walldata);
        irv.loop();
    }
    return 0;
}
