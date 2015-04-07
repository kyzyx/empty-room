#ifndef _RERENDER_H
#define _RERENDER_H
#include "wall_finder.h"
#include "meshmanager.h"
#include "solver.h"
#include "camparams.h"

#include <string>

void outputRadianceFile(std::string filename, WallFinder& wf, MeshManager& m, InverseRender& ir);
void outputPlyFile(std::string filename, WallFinder& f, MeshManager& m, InverseRender& ir);
void outputPbrtFile(std::string filename, WallFinder& wf, MeshManager& m, InverseRender& ir, Texture floortex, const CameraParams* cam, std::string floortexfilename);
#endif
