#ifndef _RERENDER_H
#define _RERENDER_H
#include "wall_finder.h"
#include "mesh.h"
#include "solver.h"
#include "colorhelper.h"

#include <string>

void outputRadianceFile(std::string filename, WallFinder& wf, Mesh& m, InverseRender& ir);
void outputPlyFile(std::string filename, WallFinder& f, Mesh& m, InverseRender& ir);
void outputPbrtFile(std::string filename, WallFinder& wf, Mesh& m, InverseRender& ir, Texture floortex, const CameraParams* cam, std::string floortexfilename);
#endif
