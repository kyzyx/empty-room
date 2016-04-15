#include "rendering/opengl_compat.h"
#include "invrenderapp.h"
#include "wallfinder/wall_finder.h"
#include "solver.h"
#include "rerender.h"
#include "datamanager/imageio.h"
#include <iostream>
#include <pcl/console/parse.h>

using namespace std;

class SolverApp : public InvrenderApp {
    public:
        SolverApp()
            : hemicuberesolution(150), discardthreshold(0.25),
              matlabfilename(""), pbrtfilename(""), rerenderedplyfilename(""), errorplyfilename(""),
              meshcolorscale(1), gamma(1),
              label(WallFinder::LABEL_WALL),
              cameranum(-1), solveTexture(false), solveLights(true),
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

            if (inputbinaryfilename.length()) {
                ir.loadVariablesBinary(alldata, inputbinaryfilename);
                if (inlightfilename.length()) {
                    readLightsFromFile(inlightfilename, ir.lightintensities);
                    ir.reloadLights();
                }
            } else {
                if (inlightfilename.length()) {
                    readLightsFromFile(inlightfilename, ir.lightintensities);
                    ir.reloadLights();
                }
                ir.computeSamples(alldata, all, numsamples, discardthreshold, false, getProgressFunction(0,2));
            }
            for (int i = 0; i < indices.size(); i++) {
                data.push_back(alldata[indices[i]]);
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
                ir.solve(labelleddata, reglambda);
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
                vector<Material> errors;
                for (int i = 0; i < mmgr->size(); i++) {
                    if (indices[wk] == i) {
                        Material res(0,0,0);
                        double currlighting[3];
                        for (int j = 0; j < rgbl.size(); j++) {
                            lightContribution(rgbl[j], currlighting, alldata[i].lightamount[j]);
                        }
                        for (int j = 0; j < 3; j++) res(j) = currlighting[j];
                        double frac = (1-alldata[i].fractionDirect)/(1-alldata[i].fractionDirect-alldata[i].fractionUnknown);
                        res += alldata[i].netIncoming*frac;
                        res = res*ir.wallMaterial;
                        colors.push_back(res);
                        Material errmat = res-alldata[i].radiosity;
                        errmat.r = abs(errmat.r);
                        errmat.g = abs(errmat.g);
                        errmat.b = abs(errmat.b);

                        meanerr += errmat.r + errmat.g + errmat.b;
                        maxerr = maxerr<errmat.r?errmat.r:maxerr;
                        maxerr = maxerr<errmat.g?errmat.g:maxerr;
                        maxerr = maxerr<errmat.b?errmat.b:maxerr;
                        errors.push_back(errmat);
                        wk++;
                    } else {
                        colors.push_back(mmgr->getMedianVertexColor(i));
                        errors.push_back(Material(200, 0, 200));
                    }
                }
                if (dorerender) mmgr->writePlyMesh(rerenderedplyfilename, colors, meshcolorscale, gamma);
                if (doerror) mmgr->writePlyMesh(errorplyfilename, errors, .5);
                cout << "Mean error: " << meanerr/(3*indices.size()) << "; " << "max error: " << maxerr << endl;
            }
            if (dointrinsic) {
                vector<Material> colors;
                int wk = 0;
                for (int i = 0; i < mmgr->NVertices(); i++) {
                    if (indices[wk] == i) {
                        double currlighting[3];
                        for (int j = 0; j < rgbl.size(); j++) {
                            lightContribution(rgbl[j], currlighting, alldata[i].lightamount[j]);
                        }
                        for (int j = 0; j < 3; j++) alldata[i].netIncoming(j) += currlighting[j];
                        alldata[i].netIncoming *= (1-alldata[i].fractionDirect)/(1-alldata[i].fractionDirect-alldata[i].fractionUnknown);
                        float r = alldata[i].radiosity.r/alldata[i].netIncoming.r;
                        float g = alldata[i].radiosity.g/alldata[i].netIncoming.g;
                        float b = alldata[i].radiosity.b/alldata[i].netIncoming.b;
                        colors.push_back(Material(r,g,b));
                    } else {
                        colors.push_back(Material(0,0,0));
                    }
                }
                mmgr->writePlyMesh(intrinsicplyfilename, colors, meshcolorscale);
            }

            // Output scene file for rerendering
            if (pbrtfilename.length()) {
                room->wallMaterial.diffuse.r = ir.materials[0].r;
                room->wallMaterial.diffuse.g = ir.materials[0].g;
                room->wallMaterial.diffuse.b = ir.materials[0].b;
                room->ceilingMaterial.diffuse.r = ir.materials[0].r;
                room->ceilingMaterial.diffuse.g = ir.materials[0].g;
                room->ceilingMaterial.diffuse.b = ir.materials[0].b;
                room->floorMaterial.diffuse.r = ir.materials[2].r;
                room->floorMaterial.diffuse.g = ir.materials[2].g;
                room->floorMaterial.diffuse.b = ir.materials[2].b;
                if (imgr && room) {
                    if (cameranum < 0) {
                        for (int i = 0; i < imgr->size(); i++) {
                            char buf[100];
                            snprintf(buf, 100, pbrtfilename.c_str(), i);
                            outputPbrtFile(
                                    buf, room, *mmgr, rgbl,
                                    imgr->getCamera(i));
                        }
                    } else {
                        outputPbrtFile(
                                pbrtfilename, room, *mmgr, rgbl,
                                imgr->getCamera(cameranum));
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
