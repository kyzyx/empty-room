#include <GL/glew.h>
#include "solver.h"
#include <random>
#include <Eigen/Dense>
#include <fstream>

using namespace std;
using namespace Eigen;

void Material::print() {
    printf("(%.3f,%.3f,%.3f)",r, g, b);
}
void InverseRender::calculate(vector<int> indices, int numsamples, double discardthreshold, int numlights) {
    if (!setupRasterizer()) {
        return;
    }

    images = new unsigned char*[2*numsamples];
    default_random_engine generator;
    uniform_int_distribution<int> dist(0, indices.size());
    for (int i = 0; i < numsamples; ++i) {
        images[2*i] = new unsigned char[3*res*res*sizeof(float)];
        images[2*i+1] = new unsigned char[3*res*res*sizeof(float)];
    }
    cout << "Inverse rendering..." << endl;
    for (int i = 0; i < numsamples; ++i) {
        int n;
        do {
            n = dist(generator);
        } while (mesh->labels[indices[n]] > 0 || mesh->samples[indices[n]].size() == 0);

        SampleData sd;
        sd.lightamount.resize(numlights);
        sd.vertexid = indices[n];
        sd.radiosity = Material(0,0,0);
        double total = 0;
        for (int j = 0; j < mesh->samples[indices[n]].size(); ++j) {
            float s =     abs(mesh->samples[indices[n]][j].dA);
            sd.radiosity.r += mesh->samples[indices[n]][j].r*s;
            sd.radiosity.g += mesh->samples[indices[n]][j].g*s;
            sd.radiosity.b += mesh->samples[indices[n]][j].b*s;
            total += s;
        }
        sd.radiosity.r /= total;
        sd.radiosity.g /= total;
        sd.radiosity.b /= total;

        sd.fractionUnknown = renderHemicube(
                mesh->getMesh()->VertexPosition(mesh->getMesh()->Vertex(indices[n])),
                mesh->getMesh()->VertexNormal(mesh->getMesh()->Vertex(indices[n])),
                sd.netIncoming, sd.lightamount, (float*)images[2*i], (float*)images[2*i+1]
        );
        if (sd.fractionUnknown > discardthreshold) {
            --i;
            continue;
        }
        data.push_back(sd);
        if (i%10 == 9) cout << "Rendered " << i+1 << "/" << numsamples << endl;
    }
    lights.resize(numlights);
}
void InverseRender::solve() {
    if (calculateWallMaterialFromUnlit()) {
        cout << "Material estimate: (" << wallMaterial(0) << "," << wallMaterial(1) << "," << wallMaterial(2) << ")" << endl;
    }
    if (data[0].lightamount.size() == 0) return;
    lights.resize(data[0].lightamount.size());
    solveLights();
    for (int i = 0; i < lights.size(); ++i) {
        cout << "Light estimate " << i << ": (" << lights[i](0)/M_PI << "," << lights[i](1)/M_PI << "," << lights[i](2)/M_PI << ")" << endl;
    }
}

bool InverseRender::calculateWallMaterialFromUnlit() {
    vector<double> estimates[3];
    vector<double> weights;
    for (int i = 0; i < data.size(); ++i) {
        bool unlit = true;
        for (int j = 0; j < data[i].lightamount.size(); ++j) {
            if (data[i].lightamount[j] > 0) {
                unlit = false;
                break;
            }
        }
        if (unlit) {
            double w = 1 - data[i].fractionUnknown;
            for (int ch = 0; ch < 3; ++ch) {
                estimates[ch].push_back(data[i].radiosity(ch)*w*w/data[i].netIncoming(ch));
            }
            weights.push_back(w);
        }
    }
    if (estimates[0].size() == 0) return false;
    cout << "Unlit estimates: " << estimates[0].size() << endl;

    double totweight = accumulate(weights.begin(), weights.end(), 0.);
    double mean[3];
    double stddev[3];
    double newmean[3];
    double count = 0;
    for (int ch = 0; ch < 3; ++ch) {
        mean[ch] = accumulate(estimates[ch].begin(), estimates[ch].end(), 0.);
        mean[ch] /= totweight;
        stddev[ch] = 0;
        newmean[ch] = 0;
    }
    for (int i = 0; i < estimates[0].size(); ++i) {
        for (int ch = 0; ch < 3; ++ch) {
            cout << estimates[ch][i] << " ";
            stddev[ch] += (estimates[ch][i]-weights[i]*mean[ch])*(estimates[ch][i]-weights[i]*mean[ch]);
        }
        cout << endl;
    }
    for (int ch = 0; ch < 3; ++ch) {
        stddev[ch] = sqrt(stddev[ch]/totweight);
    }
    for (int mult = 1; count == 0; ++mult) {
        for (int i = 0; i < estimates[0].size(); ++i) {
            int bound = 0;
            for (int ch = 0; ch < 3; ++ch) {
                if (estimates[ch][i] < mean[ch] - mult*stddev[ch]) bound = -1;
                else if (estimates[ch][i] > mean[ch] + mult*stddev[ch]) bound = 1;
            }

            if (bound < 0) continue;
            if (bound > 0) break;
            count += weights[i];
            for (int ch = 0; ch < 3; ++ch) {
                newmean[ch] += estimates[ch][i];
            }
        }
    }
    for (int ch = 0; ch < 3; ++ch) {
        wallMaterial(ch) = newmean[ch]/count;
    }
    return true;
}

void InverseRender::writeVariablesMatlab(string filename) {
    ofstream out(filename);
    out << "A = [";
    for (int i = 0; i < data.size(); ++i) {
        for (int j = 0; j < data[i].lightamount.size(); ++j) {
            out << data[i].lightamount[j];
            if (j != data[i].lightamount.size()-1) out << ",";
        }
        if (i != data.size()-1) out << ";" << endl;
    }
    out << "];" << endl;
    out << "weights = [";
    for (int i = 0; i < data.size(); ++i) {
        out << data[i].fractionUnknown;
        if (i != data.size()-1) out << ";";
    }
    out << "];" << endl;
    for (int ch = 0; ch < 3; ++ch) {
        out << "% Channel " << ch << endl;
        out << "B" << ch << " = [";
        for (int i = 0; i < data.size(); ++i) {
            out << data[i].radiosity(ch);
            if (i != data.size()-1) out << ";";
        }
        out << "];" << endl;
        out << "C" << ch << " = [";
        for (int i = 0; i < data.size(); ++i) {
            out << data[i].netIncoming(ch);
            if (i != data.size()-1) out << ";";
        }
        out << "];" << endl;
    }
}
void InverseRender::writeVariablesBinary(string filename) {
    ofstream out(filename, ofstream::binary);
    uint32_t sz = data.size();
    out.write((char*) &sz, 4);
    sz = data[0].lightamount.size();
    out.write((char*) &sz, 4);
    for (int i = 0; i < data.size(); ++i) {
        out.write((char*)&(data[i].fractionUnknown), sizeof(float));
        out.write((char*)&(data[i].vertexid), sizeof(int));
        out.write((char*)&(data[i].radiosity(0)), sizeof(float));
        out.write((char*)&(data[i].radiosity(1)), sizeof(float));
        out.write((char*)&(data[i].radiosity(2)), sizeof(float));
        out.write((char*)&(data[i].netIncoming(0)), sizeof(float));
        out.write((char*)&(data[i].netIncoming(1)), sizeof(float));
        out.write((char*)&(data[i].netIncoming(2)), sizeof(float));
        for (int j = 0; j < data[i].lightamount.size(); ++j) {
            out.write((char*)&(data[i].lightamount[j]), sizeof(float));
        }
    }
}

void InverseRender::loadVariablesBinary(string filename) {
    ifstream in(filename, ifstream::binary);
    uint32_t sz;
    in.read((char*) &sz, 4);
    data.resize(sz);
    in.read((char*) &sz, 4);
    for (int i = 0; i < data.size(); ++i) {
        data[i].lightamount.resize(sz);
        in.read((char*)&(data[i].fractionUnknown), sizeof(float));
        in.read((char*)&(data[i].vertexid), sizeof(int));
        in.read((char*)&(data[i].radiosity(0)), sizeof(float));
        in.read((char*)&(data[i].radiosity(1)), sizeof(float));
        in.read((char*)&(data[i].radiosity(2)), sizeof(float));
        in.read((char*)&(data[i].netIncoming(0)), sizeof(float));
        in.read((char*)&(data[i].netIncoming(1)), sizeof(float));
        in.read((char*)&(data[i].netIncoming(2)), sizeof(float));
        for (int j = 0; j < data[i].lightamount.size(); ++j) {
            in.read((char*)&(data[i].lightamount[j]), sizeof(float));
        }
    }
}

bool InverseRender::setupRasterizer() {
    glClampColor(GL_CLAMP_READ_COLOR, GL_FALSE);
    glClampColor(GL_CLAMP_VERTEX_COLOR, GL_FALSE);
    glClampColor(GL_CLAMP_FRAGMENT_COLOR, GL_FALSE);

    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, res, res, 0, GL_RGBA, GL_FLOAT, 0);
    glBindTexture(GL_TEXTURE_2D, 0);

    glGenRenderbuffers(1, &fbo_z);
    glBindRenderbuffer(GL_RENDERBUFFER_EXT, fbo_z);
    glRenderbufferStorage(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT, res, res);
    glBindRenderbuffer(GL_RENDERBUFFER_EXT, 0);

    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER_EXT, fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0);
    //glFramebufferRenderbuffer(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER_EXT, fbo_rgb);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, fbo_z);
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER_EXT);
    if (status != GL_FRAMEBUFFER_COMPLETE_EXT) {
        cout << "Error creating frame buffer" << endl;
        return false;
    }
    glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0);
    return true;
}

const double threshold = 0.01; // % change above which we are not converged
bool InverseRender::solveLights() {
    // Weighted linear least squares for each channel
    double maxchange = 0.;
    int numlights = data[0].lightamount.size();
    for (int ch = 0; ch < 3; ++ch) {
        VectorXd b(data.size());
        MatrixXd A(data.size(), numlights);
        for (int i = 0; i < data.size(); ++i) {
            b[i] = (1-data[i].fractionUnknown)*data[i].radiosity(ch) - wallMaterial(ch)*data[i].netIncoming(ch);
            for (int j = 0; j < numlights; ++j) {
                A(i,j) = (1-data[i].fractionUnknown)*wallMaterial(ch)*data[i].lightamount[j];
            }
        }
        VectorXd x = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
        for (int i = 0; i < numlights; ++i) {
            if (lights[i](ch) > 0) {
                double delta = abs(lights[i](ch) - x[i]);
                double percent = delta/lights[i](ch);
                if (percent > maxchange) maxchange = percent;
            }
            lights[i](ch) = x[i];
        }
    }
    return maxchange < threshold;
}

bool InverseRender::solveMaterials() {
    // Weighted average over sample points
    bool converged = true;
    int numlights = data[0].lightamount.size();
    for (int ch = 0; ch < 3; ++ch) {
        double prev = wallMaterial(ch);
        wallMaterial(ch) = 0;
        double tot = 0;
        for (int i = 0; i < data.size(); ++i) {
            double totalin = data[i].netIncoming(ch);
            for (int j = 0; j < numlights; ++j) {
                totalin += lights[j](ch)*data[i].lightamount[j]*(1-data[i].fractionUnknown);
            }
            double p = data[i].radiosity(ch)*(1-data[i].fractionUnknown)/totalin;
            wallMaterial(ch) += p;
            tot += 1-data[i].fractionUnknown;
        }
        wallMaterial(ch) /= tot;
        if (abs(prev - wallMaterial(ch)) > threshold) converged = false;
    }
    return converged;
}
void InverseRender::computeHemicubeFF() {
    // Some redundancy, but storage is cheap
    topHemicubeFF = new float*[res];
    sideHemicubeFF = new float*[res];
    for (int i = 0; i < res; ++i) {
        topHemicubeFF[i] = new float[res];
        sideHemicubeFF[i] = new float[res];
    }
    float x = 1./res;
    float d = 2./res;
    int o = res/2;
    float tot = 0;
    for (int i = 1; i <= res/2; ++i, x += d) {
        float y = 1./res;
        for (int j = 1; j <= i; ++j, y += d) {
            float denom = x*x + y*y + 1;
            topHemicubeFF[o+i-1][o+j-1] = d*d/(M_PI*denom*denom);
            topHemicubeFF[o+i-1][o-j]   = topHemicubeFF[o+i-1][o+j-1];
            topHemicubeFF[o-i][o-j]     = topHemicubeFF[o+i-1][o+j-1];
            topHemicubeFF[o-i][o+j-1]   = topHemicubeFF[o+i-1][o+j-1];
            topHemicubeFF[o+j-1][o+i-1] = topHemicubeFF[o+i-1][o+j-1];
            topHemicubeFF[o+j-1][o-i]   = topHemicubeFF[o+i-1][o+j-1];
            topHemicubeFF[o-j][o-i]     = topHemicubeFF[o+i-1][o+j-1];
            topHemicubeFF[o-j][o+i-1]   = topHemicubeFF[o+i-1][o+j-1];
        }
        float z = 1./res;
        for (int j = 1; j <= res/2; ++j, z += d) {
            float denom = z*z + x*x + 1;
            sideHemicubeFF[o+i-1][o+j-1] = d*d*z/(M_PI*denom*denom);
            sideHemicubeFF[o+i-1][o-j] = sideHemicubeFF[o+i-1][o+j-1];
            sideHemicubeFF[o-i][o-j]   = sideHemicubeFF[o+i-1][o+j-1];
            sideHemicubeFF[o-i][o+j-1] = sideHemicubeFF[o+i-1][o+j-1];
        }
    }
}
float InverseRender::renderHemicube(
        const R3Point& p,
        const R3Vector& n,
        Material& m,
        vector<float>& lightareas,
        float* image, float* light)
{
    float blank = 0;
    if (image == NULL) image = new float[3*res*res];
    if (light == NULL) light = new float[3*res*res];
    R3Point pp = p + 0.0001*n;
    R3Vector x = R3yaxis_vector;
    x.Cross(n);
    R3Vector y = x%n;
    R3Vector orientations[] = {x,-x,y,-y};
    for (int o = 0; o < 4; ++o) {
        renderFace(pp, orientations[o], n, image, true);
        renderFace(pp, orientations[o], n, light, false);
        for (int i = res/2; i < res; ++i) {
            for (int j = 0; j < res; ++j) {
                if (light[3*(i*res+j)] != 0 &&
                        light[3*(i*res+j)+1] == 0 &&
                        light[3*(i*res+j)+2] == 0)
                {
                    int lightid = light[3*(i*res+j)]*MAX_LIGHTS;
                    lightareas[lightid] += sideHemicubeFF[i][j];
                }  else if (light[3*(i*res+j)] == 0 &&
                        light[3*(i*res+j)+1] == 0 &&
                        light[3*(i*res+j)+2] != 0)
                {
                    blank += sideHemicubeFF[i][j];
                } else {
                    m.r += sideHemicubeFF[i][j]*image[3*(i*res+j)];
                    m.g += sideHemicubeFF[i][j]*image[3*(i*res+j)+1];
                    m.b += sideHemicubeFF[i][j]*image[3*(i*res+j)+2];
                }
            }
        }
    }
    renderFace(pp, n, y, image, true);
    renderFace(pp, n, y, light, false);
    for (int i = 0; i < res; ++i) {
        for (int j = 0; j < res; ++j) {
            if (light[3*(i*res+j)] != 0 &&
                    light[3*(i*res+j)+1] == 0 &&
                    light[3*(i*res+j)+2] == 0)
            {
                lightareas[light[3*(i*res+j)]-1] += topHemicubeFF[i][j];
            }  else if (light[3*(i*res+j)] == 0 &&
                    light[3*(i*res+j)+1] == 0 &&
                    light[3*(i*res+j)+2] != 0)
            {
                blank += topHemicubeFF[i][j];
            } else {
                m.r += topHemicubeFF[i][j]*image[3*(i*res+j)];
                m.g += topHemicubeFF[i][j]*image[3*(i*res+j)+1];
                m.b += topHemicubeFF[i][j]*image[3*(i*res+j)+2];
            }
        }
    }
    return blank;
}
void InverseRender::renderFace(const R3Point& p,
        const R3Vector& towards, const R3Vector& up,
        float* image, bool colorimage)
{
    glDisable(GL_CULL_FACE);
    glViewport(0,0,res,res);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(90, 1., 0.001, 100);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    R3Point at = p + towards;
    gluLookAt(p[0], p[1], p[2], at[0], at[1], at[2], up[0], up[1], up[2]);
    glBindFramebuffer(GL_FRAMEBUFFER_EXT, fbo);
    mesh->renderOGL(colorimage);
    glReadPixels(0,0,res,res,GL_RGB,GL_FLOAT,(void*)image);
    glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0);
}
