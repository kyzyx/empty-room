#ifndef _ROOMMODEL_H
#define _ROOMMODEL_H

#define ROOMVISUALIZER_API
#define UPROPERTY(vis, access, cat)
#define FVector R3Vector
#define FMatrix R4Matrix
#define IdentityMatrix R4identity_matrix
#define getMatrixElement(m,i,j) m[i][j]
#define setMatrixElement(m,i,j,d) m[i][j] = d
#include "R3Shapes/R3Shapes.h"
#include "RoomModel_Internal.h"
#endif
