#ifndef _FIND_BASEBOARD_H
#define _FIND_BASEBOARD_H

#include "datamanager/imagemanager.h"
#include "roommodel/roommodel.h"

class BaseboardFinder {
    public:
        BaseboardFinder(
                ImageManager* imagemanager,
                roommodel::RoomModel* room)
            : imgr(imagemanager), floorplan(room), solved(false)
        { ; }

        void compute(float resolution=0.02, float edgethreshold=6);

        roommodel::Color getBaseboardColor() {
            if (!solved) compute();
            return bbcolor;
        }
        float getBaseboardHeight() {
            if (!solved) compute();
            return bbh;
        }
        float getBaseboardDepth() {
            if (!solved) compute();
            return bbd;
        }

    protected:
        ImageManager* imgr;
        roommodel::RoomModel* floorplan;

        float bbh, bbd;
        roommodel::Color bbcolor;
        bool solved;
};

#endif
