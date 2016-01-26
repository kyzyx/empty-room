#ifndef _RERENDER_H
#define _RERENDER_H
#include "wallfinder/wall_finder.h"
#include "datamanager/meshmanager.h"
#include "solver.h"
#include "datamanager/camparams.h"
#include "roommodel/roommodel.h"

#include <string>

void outputRadianceFile(std::string filename, WallFinder& wf, MeshManager& m, InverseRender& ir);
void outputPlyFile(std::string filename, WallFinder& f, MeshManager& m, InverseRender& ir);
void outputPbrtFile(std::string filename, roommodel::RoomModel* room, MeshManager& m, std::vector<Material>& lights, const CameraParams* cam);
#endif
