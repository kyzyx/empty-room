#include "rendering/opengl_compat.h"
#include "findbaseboard.h"
#include "featurefinder.h"
#include "linefinder.h"
#include "invrenderapp.h"
#include "wallfinder/wall_finder.h"
#include "solver.h"
#include "rerender.h"
#include "datamanager/imageio.h"
#include <iostream>
#include <pcl/console/parse.h>

using namespace std;
using namespace Eigen;

Matrix3d getRotationMatrix(const CameraParams* cam) {
    Matrix3d ret;
    ret << cam->right[0], cam->towards[0], cam->up[0],
           cam->right[1], cam->towards[1], cam->up[1],
           cam->right[2], cam->towards[2], cam->up[2];
    return ret;
}
CameraParams fromRotationMatrix(Eigen::Matrix3d m) {
    CameraParams ret;
    Vector3d right = m*Vector3d(1,0,0);
    Vector3d up = m*Vector3d(0,0,1);
    Vector3d towards = m*Vector3d(0,1,0);
    ret.up = R3Vector(up[0], up[1], up[2]);
    ret.right = R3Vector(right[0], right[1], right[2]);
    ret.towards = R3Vector(towards[0], towards[1], towards[2]);
    return ret;
}


class SolverApp : public InvrenderApp {
    public:
        SolverApp()
            : hemicuberesolution(150), discardthreshold(0.25),
              matlabfilename(""), pbrtfilename(""), rerenderedplyfilename(""), errorplyfilename(""),
              meshcolorscale(1), gamma(1),
              label(WallFinder::LABEL_WALL),
              cameranum(-1), solveTexture(false), solveLights(true), dobaseboard(false), dorwo(false),
              selectlight(-1), selectlightthresh(0.001), selectlightcoef(-1),
              scale(0), reglambda(0) {}
        virtual int run() {
            // Setup
            mmgr->loadSamples();
            InverseRender ir(mmgr, hemicuberesolution, getProgressFunction(1,2));

            // Populate wall data
            vector<SampleData> data, alldata;
            vector<int> indices, all;
            for (int i = 0; i < mmgr->size(); i++) {
                all.push_back(i);
                if (label < 0 || mmgr->getLabel(i, MeshManager::TYPE_CHANNEL) == label)
                    indices.push_back(i);
            }

            if (inlightfilename.length()) {
                readLightsFromFile(inlightfilename, ir.lightintensities);
                ir.reloadLights();
            }
            if (inputbinaryfilename.length()) {
                ir.loadVariablesBinary(alldata, inputbinaryfilename);
                if (inlightfilename.length()) {
                    readLightsFromFile(inlightfilename, ir.lightintensities);
                    ir.reloadLights();
                }
            } else {
                ir.computeSamples(alldata, all, numsamples, discardthreshold, false, getProgressFunction(0,2));
            }
            if (room) {
                ir.materials.resize(3);
                ir.materials[0].r = room->wallMaterial.diffuse.r   ;
                ir.materials[0].g = room->wallMaterial.diffuse.g   ;
                ir.materials[0].b = room->wallMaterial.diffuse.b   ;
                ir.materials[1].r = room->ceilingMaterial.diffuse.r;
                ir.materials[1].g = room->ceilingMaterial.diffuse.g;
                ir.materials[1].b = room->ceilingMaterial.diffuse.b;
                ir.materials[2].r = room->floorMaterial.diffuse.r  ;
                ir.materials[2].g = room->floorMaterial.diffuse.g  ;
                ir.materials[2].b = room->floorMaterial.diffuse.b  ;
            }
            for (int i = 0; i < indices.size(); i++) {
                data.push_back(alldata[indices[i]]);
            }
            if (selectlight > 0) {
                for (int i = 0; i < mmgr->size(); i++) {
                    Light* l = alldata[i].lightamount[selectlight-1];
                    double tot = 0;
                    if (selectlightcoef >= 0) {
                        tot += l->getCoef(selectlightcoef);
                    } else {
                        for (int j = 0; j < l->numParameters(); j++) {
                            tot += l->getCoef(j);
                        }
                    }
                    if (tot > selectlightthresh) {
                        cout << ">>data:" << i << endl;
                    }
                }
                cout << ">>done" << endl;

                return 0;
            } else if (selectlight == 0) {
                for (int i = 0; i < mmgr->size(); i++) {
                    double meanexitant = alldata[i].radiosity.r + alldata[i].radiosity.g + alldata[i].radiosity.b;
                    double meanincident = alldata[i].netIncoming.r + alldata[i].netIncoming.g + alldata[i].netIncoming.b;
                    meanincident *= reweightIncoming(alldata[i]);
                    if (meanexitant > 0.8*meanincident) cout << ">>data:" << i << endl;
                }
                cout << ">>done" << endl;
            }

            // Optimization
            if (solveTexture) {
                Texture tex;
                R3Vector v(0,0,0);
                R3Point p(0,0,0);
                for (int i = 0; i < indices.size(); i++) {
                    v += mmgr->VertexNormal(indices[i]);
                    p += mmgr->VertexPosition(indices[i]);
                }
                v.Normalize();
                p /= indices.size();

                R3Plane plane(p, v);
                ir.solveTexture(data, imgr, plane, tex, label);
                if (tex.texture) ImageIO::writeExrImage(texturefilename, tex.texture, tex.size, tex.size);
            } else if (solveLights) {
                if (scale > 0) {
                    ir.setLossFunction(LOSS_HUBER);
                    ir.setLossFunctionScale(scale);
                }
                vector<SampleData> labelleddata;
                for (int i = 0; i < mmgr->size(); i++) {
                    if (label < 0 || mmgr->getLabel(i, MeshManager::TYPE_CHANNEL) > 0)
                        labelleddata.push_back(alldata[i]);
                }
                for (int i = 0; i < ir.lightintensities.size(); i++) {
                    ir.lightintensities[i]->setRegularization(reglambda);
                }
                ir.solve(labelleddata, reglambda, true);
            }
            vector<Light*> rgbl;
            for (int i = 0; i < ir.lightintensities.size(); i++) {
                if (ir.lightintensities[i]->typeId() & LIGHTTYPE_RGB) {
                    rgbl.push_back(ir.lightintensities[i]);
                } else if (ir.lightintensities[i]->typeId() & LIGHTTYPE_IRGB) {
                    rgbl.push_back(((IRGBLight*) ir.lightintensities[i])->toRGBLight());
                }
            }

            // Output optimization data and results
            if (matlabfilename.length()) {
                ir.writeVariablesMatlab(data, matlabfilename);
            }
            if (outputbinaryfilename.length()) {
                ir.writeVariablesBinary(alldata, outputbinaryfilename);
            }
            if (solveTexture) return 0;
            if (outlightfilename.length()) {
                writeLightsToFile(outlightfilename, ir.lightintensities);
            }
            if (room) {
                if (ir.materials.size() >= 1) {
                    room->wallMaterial.diffuse.r = ir.materials[0].r;
                    room->wallMaterial.diffuse.g = ir.materials[0].g;
                    room->wallMaterial.diffuse.b = ir.materials[0].b;
                    room->ceilingMaterial.diffuse.r = ir.materials[1].r;
                    room->ceilingMaterial.diffuse.g = ir.materials[1].g;
                    room->ceilingMaterial.diffuse.b = ir.materials[1].b;
                    room->floorMaterial.diffuse.r = ir.materials[2].r;
                    room->floorMaterial.diffuse.g = ir.materials[2].g;
                    room->floorMaterial.diffuse.b = ir.materials[2].b;
                }
            }
            for (int i = 0; i < ir.materials.size(); i++) {
                cout << "data:Material " << i << " ";
                cout << ir.materials[i].r << " " << ir.materials[i].g << " " << ir.materials[i].b << endl;
            }
            for (int i = 0; i < rgbl.size(); i++) {
                cout << "data:Light " << i << " ";
                rgbl[i]->writeToStream(cout);
                cout << endl;
            }

            // Output debugging results of optimization
            bool dorerender = rerenderedplyfilename.length()>0;
            bool doerror = errorplyfilename.length()>0;
            bool dointrinsic = intrinsicplyfilename.length()>0;
            double meanerr = 0;
            double maxerr = 0;
            if (dorerender || doerror) {
                int wk = 0;
                vector<Material> colors;
                vector<Material> poserrors;
                vector<Material> negerrors;
                for (int i = 0; i < mmgr->size(); i++) {
                    if (wk < indices.size() && indices[wk] == i) {
                        Material res = computeIncident(alldata[i], rgbl);
                        res = res*ir.materials[label-1];
                        colors.push_back(res);
                        Material errmat = res-alldata[i].radiosity;
                        cout << errmat.r << " " << errmat.g << " " << errmat.b << endl;
                        Material perrm(
                                errmat.r>0?errmat.r:0,
                                errmat.g>0?errmat.g:0,
                                errmat.b>0?errmat.b:0);
                        Material nerrm(
                                errmat.r<0?errmat.r:0,
                                errmat.g<0?errmat.g:0,
                                errmat.b<0?errmat.b:0);
                        errmat.r = abs(errmat.r);
                        errmat.g = abs(errmat.g);
                        errmat.b = abs(errmat.b);

                        meanerr += errmat.r + errmat.g + errmat.b;
                        maxerr = maxerr<errmat.r?errmat.r:maxerr;
                        maxerr = maxerr<errmat.g?errmat.g:maxerr;
                        maxerr = maxerr<errmat.b?errmat.b:maxerr;
                        poserrors.push_back(perrm);
                        negerrors.push_back(nerrm);
                        wk++;
                    } else {
                        colors.push_back(alldata[i].radiosity);
                        poserrors.push_back(Material(200, 0, 200));
                        negerrors.push_back(Material(200, 0, 200));
                    }
                }
                if (dorerender) mmgr->writePlyMesh(rerenderedplyfilename, colors, meshcolorscale, gamma);
                if (doerror) {
                    mmgr->writePlyMesh("pos" + errorplyfilename, poserrors, 25);
                    mmgr->writePlyMesh("neg" + errorplyfilename, negerrors, 25);
                }
                cout << "Mean error: " << meanerr/(3*indices.size()) << "; " << "max error: " << maxerr << endl;
            }
            if (dointrinsic) {
                vector<Material> colors;
                int wk = 0;
                for (int i = 0; i < mmgr->NVertices(); i++) {
                    //if (indices[wk] == i) {
                        Material m = computeIncident(alldata[i], rgbl);
                        float r = alldata[i].radiosity.r/m.r;
                        float g = alldata[i].radiosity.g/m.g;
                        float b = alldata[i].radiosity.b/m.b;
                        colors.push_back(Material(r,g,b));
                    //} else {
                        //colors.push_back(Material(0,0,0));
                    //}
                }
                mmgr->writePlyMesh(intrinsicplyfilename, colors, meshcolorscale);
            }
            if (dobaseboard) {
                FloorplanHelper fph;
                fph.loadFromRoomModel(room);
                BaseboardFinder bf(imgr, room);
                //bf.compute();
                //double bh = bf.getBaseboardHeight();
                double bh = 0.06;
                FeatureFinder ff;
                ff.condition.setLabel(WallFinder::LABEL_WALL);
                ff.condition.setMax(1, bh);
                ff.compute(fph, ir, alldata);
                Material bbmat = ff.getMaterial();
                room->baseboardMaterial.diffuse.r = bbmat.r;
                room->baseboardMaterial.diffuse.g = bbmat.g;
                room->baseboardMaterial.diffuse.b = bbmat.b;
                room->baseboardHeight = bh;
                room->baseboardDepth = ff.getDepth();
                if (room->baseboardDepth < 0) room->baseboardDepth = 0.002;
            }
            if (dorwo) {
                FloorplanHelper fph;
                fph.loadFromRoomModel(room);
                vector<WallLine> lines;
                //findWallLines(*imgr, fph, lines, 0.03);
                WallLine wl(2, 0.104314);
                wl.endy = 0.0410754;
                wl.starty = 2.06108;
                lines.push_back(wl);
                WallLine wl2(2, 1.03457);
                wl2.endy = 0.0710754;
                wl2.starty = 2.13108;
                lines.push_back(wl2);
                WallLine wl3(3, 0.189039);
                wl3.endy = -0.0426901;
                wl3.starty = 2.26731;
                lines.push_back(wl3);
                WallLine wl4(3, 2.55969);
                wl4.endy = -0.0426901;
                wl4.starty = 2.26731;
                lines.push_back(wl4);
                WallLine hl(3, 2.28);
                hl.endy = 0.189039;
                hl.starty = 2.557;
                hl.vertical = false;
                lines.push_back(hl);
                WallLine hl2(2, 2.10189);
                hl2.endy = 0.0;
                hl2.starty = 1.08039;
                hl2.vertical = false;
                lines.push_back(hl2);
                // Find all potential door vertical lines
                double dh = 0.08;
                double mindoorheight = 1.7;
                vector<vector<WallLine> > perwalllines(fph.wallsegments.size());
                vector<vector<WallLine> > perwallhoriz(fph.wallsegments.size());
                for (int i = 0; i < lines.size(); i++) {
                    if (lines[i].vertical) {
                        if (
                                lines[i].endy < dh
                             && lines[i].starty > mindoorheight
                        ) {
                            perwalllines[lines[i].wallindex].push_back(lines[i]);
                        cout << "Potential door line " << lines[i].wallindex << " " << lines[i].endy << " " << lines[i].starty << " " << lines[i].p << endl;
                        }
                    } else {
                        perwallhoriz[lines[i].wallindex].push_back(lines[i]);
                    }
                }
                for (int i = 0; i < perwalllines.size(); i++) {
                    vector<bool> considered(perwalllines[i].size(),false);
                    for (int j = 0; j < perwalllines[i].size(); j++) {
                        double h = perwalllines[i][j].starty;
                        double p = perwalllines[i][j].p;
                        int bestk = -1;
                        double maxw = 0;
                        if (considered[j]) continue;
                        considered[j] = true;
                        for (int k = j+1; k < perwalllines[i].size(); k++) {
                            double h2 = perwalllines[i][k].starty;
                            double p2 = perwalllines[i][k].p;
                            bool compatible = false;
                            for (int x = 0; x < perwallhoriz[i].size(); x++) {
                                double ph = perwallhoriz[i][x].p;
                                double pl = perwallhoriz[i][x].endy;
                                double pr = perwallhoriz[i][x].starty;
                                double ppl = min(p,p2);
                                double ppr = max(p,p2);
                                if (abs(ph-h) < dh
                                        && pl < ppl + dh && pr > ppr - dh)
                                        //&& pl < p + dh && pr > p - dh
                                        //&& pl < p2 + dh && pr > p2 - dh)
                                {
                                    compatible = true;
                                    cout << h << " " << h2 << " " << perwallhoriz[i][x].p << " " << perwallhoriz[i][x].endy << " " << perwallhoriz[i][x].starty << endl;
                                }
                            }
                            if (abs(h-h2) < dh && compatible) {
                                double currw = abs(p-p2);
                                considered[k] = true;
                                if (currw > maxw) {
                                    maxw = currw;
                                    bestk = k;
                                }
                            }
                        }
                        if (bestk >= 0) {
                            double h2 = perwalllines[i][bestk].starty;
                            double p2 = perwalllines[i][bestk].p;
                            FeatureFinder ff;
                            ff.condition.setLabel(WallFinder::LABEL_WALL);
                            ff.condition.setWallIndex(i);
                            ff.condition.setMax(1, (h+h2)/2);
                            if (fph.wallsegments[i].direction) {
                                ff.condition.setMin(0, min(p,p2) + fph.wallsegments[i].start);
                                ff.condition.setMax(0, max(p,p2) + fph.wallsegments[i].start);
                            } else {
                                ff.condition.setMin(2, min(p,p2) + fph.wallsegments[i].start);
                                ff.condition.setMax(2, max(p,p2) + fph.wallsegments[i].start);
                            }
                            ff.compute(fph, ir, alldata);
                            Material doormat = ff.getMaterial();
                            roommodel::RectangleWallObject rwo;
                            rwo.material.diffuse.r = doormat.r;
                            rwo.material.diffuse.g = doormat.g;
                            rwo.material.diffuse.b = doormat.b;
                            rwo.height = h;
                            rwo.width = abs(p - p2);
                            rwo.recessed = -ff.getDepth();
                            cout << fph.wallsegments[i].norm << " " << fph.wallsegments[i].direction << " " <<fph.forwards[i] << " " << fph.wallsegments[i].start << " " << fph.wallsegments[i].end << " " << p << " " << p2 << " " << -ff.getDepth() << endl;
                            rwo.verticalposition = 0.0001; // Must be > 0 for geometry generator
                            int nn = fph.wallsegments[i].norm>0?1:0;
                            if (fph.forwards[i] != (fph.wallsegments[i].norm < 0)) {
                                rwo.horizontalposition = fph.wallsegments[i].length() - max(p,p2);
                            } else {
                                rwo.horizontalposition = min(p,p2);
                            }
                            rwo.trimDepth = 0;
                            rwo.trimWidth = 0;
                            room->walls[i].windows.push_back(rwo);
                        }
                    }
                }
            }
            if (outroommodelname.length()) {
                roommodel::save(*room, outroommodelname);
            }

            // Output scene file for rerendering
            if (pbrtfilename.length()) {
                if (imgr && room) {
                    outputPbrtFile(
                            pbrtfilename+".pbrt", room, *mmgr, rgbl,
                            cameranum<0?NULL:imgr->getCamera(cameranum)
                    );
                    if (cameranum < 0) {
                        for (int i = 0; i < imgr->size(); i++) {
                            char buf[100];
                            snprintf(buf, 100, "%s%04d.pbrt", pbrtfilename.c_str(), i);
                            outputPbrtCameraFile(
                                    buf, pbrtfilename+".pbrt",
                                    imgr->getCamera(i));
                            if (i) {
                                // Output interpolated camera position
                                CameraParams cam;
                                Eigen::Quaterniond q(getRotationMatrix(imgr->getCamera(i)));
                                Eigen::Quaterniond q2(getRotationMatrix(imgr->getCamera(i-1)));
                                Eigen::Quaterniond q3 = q2.slerp(0.5,q);
                                cam = fromRotationMatrix(q3.toRotationMatrix());
                                cam.pos = (imgr->getCamera(i)->pos + imgr->getCamera(i-1)->pos)/2;
                                cam.gamma = imgr->getCamera(i)->gamma;
                                cam.width = imgr->getCamera(i)->width;
                                cam.height = imgr->getCamera(i)->height;
                                cam.focal_length = imgr->getCamera(i)->focal_length;
                                cam.fov = imgr->getCamera(i)->fov;
                                snprintf(buf, 100, "%s%04d_5.pbrt", pbrtfilename.c_str(), i-1);
                                outputPbrtCameraFile(
                                        buf, pbrtfilename+".pbrt",
                                        &cam);
                            }
                        }
                    }
                } else {
                    cout << "Error - need to supply imagemanager and roommodel to output pbrt file" << endl;
                }
            }
            return 0;
        }
    protected:
        virtual void printargs() {

        }
        virtual int _parseargs(int argc, char** argv) {
            if (!mmgr) return 0;
            if (pcl::console::find_switch(argc, argv, "-nosolve")) {
                solveLights = false;
            }
            if (pcl::console::find_switch(argc, argv, "-baseboard")) {
                dobaseboard = true;
            }
            if (pcl::console::find_switch(argc, argv, "-doors")) {
                dorwo = true;
            }
            if (pcl::console::find_argument(argc, argv, "-hemicuberesolution") >= 0) {
                pcl::console::parse_argument(argc, argv, "-hemicuberesolution", hemicuberesolution);
            }
            if (pcl::console::find_argument(argc, argv, "-label") >= 0) {
                pcl::console::parse_argument(argc, argv, "-label", label);
            }
            if (pcl::console::find_argument(argc, argv, "-discardthreshold") >= 0) {
                pcl::console::parse_argument(argc, argv, "-discardthreshold", discardthreshold);
            }
            if (pcl::console::find_argument(argc, argv, "-numsamples") >= 0) {
                pcl::console::parse_argument(argc, argv, "-numsamples", numsamples);
            }
            if (pcl::console::find_argument(argc, argv, "-scale") >= 0) {
                pcl::console::parse_argument(argc, argv, "-scale", scale);
            }
            if (pcl::console::find_argument(argc, argv, "-outputsamplesfile") >= 0) {
                pcl::console::parse_argument(argc, argv, "-outputsamplesfile", matlabfilename);
            }
            if (pcl::console::find_argument(argc, argv, "-inputsamplesfile") >= 0) {
                pcl::console::parse_argument(argc, argv, "-inputsamplesfile", inputmatlabfilename);
            }
            if (pcl::console::find_argument(argc, argv, "-outputbinaryfile") >= 0) {
                pcl::console::parse_argument(argc, argv, "-outputbinaryfile", outputbinaryfilename);
            }
            if (pcl::console::find_argument(argc, argv, "-inputbinaryfile") >= 0) {
                pcl::console::parse_argument(argc, argv, "-inputbinaryfile", inputbinaryfilename);
            }
            if (pcl::console::find_argument(argc, argv, "-inputlightfile") >= 0) {
                pcl::console::parse_argument(argc, argv, "-inputlightfile", inlightfilename);
            }
            if (pcl::console::find_argument(argc, argv, "-outputlightfile") >= 0) {
                pcl::console::parse_argument(argc, argv, "-outputlightfile", outlightfilename);
            }
            if (pcl::console::find_argument(argc, argv, "-outputpbrtfile") >= 0) {
                pcl::console::parse_argument(argc, argv, "-outputpbrtfile", pbrtfilename);
            }
            if (pcl::console::find_argument(argc, argv, "-cameraid") >= 0) {
                pcl::console::parse_argument(argc, argv, "-cameraid", cameranum);
            }
            if (pcl::console::find_argument(argc, argv, "-lambda") >= 0) {
                pcl::console::parse_argument(argc, argv, "-lambda", reglambda);
            }
            if (pcl::console::find_argument(argc, argv, "-outputcolorscale") >= 0) {
                pcl::console::parse_argument(argc, argv, "-outputcolorscale", meshcolorscale);
            }
            if (pcl::console::find_argument(argc, argv, "-outputgamma") >= 0) {
                pcl::console::parse_argument(argc, argv, "-outputgamma", gamma);
            }
            if (pcl::console::find_argument(argc, argv, "-outputtexture") >= 0) {
                pcl::console::parse_argument(argc, argv, "-outputtexture", texturefilename);
                solveTexture = true;
            }
            if (pcl::console::find_argument(argc, argv, "-outputerrormesh") >= 0) {
                pcl::console::parse_argument(argc, argv, "-outputerrormesh", errorplyfilename);
            }
            if (pcl::console::find_argument(argc, argv, "-outputrerendermesh") >= 0) {
                pcl::console::parse_argument(argc, argv, "-outputrerendermesh", rerenderedplyfilename);
            }
            if (pcl::console::find_argument(argc, argv, "-outputintrinsicmesh") >= 0) {
                pcl::console::parse_argument(argc, argv, "-outputintrinsicmesh", intrinsicplyfilename);
            }
            if (pcl::console::find_argument(argc, argv, "-outputroommodel") >= 0) {
                pcl::console::parse_argument(argc, argv, "-outputroommodel", outroommodelname);
            }
            if (pcl::console::find_argument(argc, argv, "-selectlight") >= 0) {
                pcl::console::parse_argument(argc, argv, "-selectlight", selectlight);
            }
            if (pcl::console::find_argument(argc, argv, "-selectlightthreshold") >= 0) {
                pcl::console::parse_argument(argc, argv, "-selectlightthreshold", selectlightthresh);
            }
            if (pcl::console::find_argument(argc, argv, "-selectlightcoef") >= 0) {
                pcl::console::parse_argument(argc, argv, "-selectlightcoef", selectlightcoef);
            }
            return true;
        }
        int numsamples;
        float discardthreshold;
        float scale;
        int hemicuberesolution;
        string matlabfilename;
        string inputmatlabfilename;
        string outputbinaryfilename;
        string inputbinaryfilename;
        string pbrtfilename;
        string rerenderedplyfilename;
        string errorplyfilename;
        string intrinsicplyfilename;
        string outlightfilename;
        string inlightfilename;
        string outroommodelname;
        int selectlight;
        int selectlightcoef;
        double selectlightthresh;
        bool dobaseboard;
        bool dorwo;
        int cameranum;
        int label;
        double reglambda;
        float meshcolorscale;
        float gamma;
        bool solveTexture;
        bool solveLights;
        string texturefilename;
};

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(1,1);
    glutCreateWindow("");
    glutHideWindow();
    SolverApp app;
    if (!app.parseargs(argc, argv)) return 1;
    return app.run();
}
