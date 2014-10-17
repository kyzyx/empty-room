#include "util.h"
#include <limits>
#include <pcl/common/common.h>
#include <pcl/point_representation.h>

using namespace Eigen;
using namespace pcl;
using namespace std;

static DefaultPointRepresentation<PointXYZ> pr;
bool isValid(PointXYZ p) { return pr.isValid(p); }
double dist2(PointXYZ a, PointXYZ b) {
    if (!pr.isValid(a) || !pr.isValid(b)) return numeric_limits<double>::infinity();
    return (Vector3f(a.x,a.y,a.z)-Vector3f(b.x,b.y,b.z)).squaredNorm();
}

Vector4d transformPlane(Vector4d plane, Matrix4d transform)
{
    Vector4d dir = plane;
    dir(3) = 0;
    Vector4d p(-plane(3)*plane(0), -plane(3)*plane(1), -plane(3)*plane(2), 1);

    dir = transform*dir;
    dir.head(3) = dir.head(3).normalized();
    p = transform*p;

    dir(3) = -dir.dot(p);
    return dir;
}

Vector3d cloudMidpoint(PointCloud<PointXYZ>::ConstPtr cloud)
{
    PointXYZ minp, maxp;
    getMinMax3D(*cloud, minp, maxp);
    Vector3d minpt(minp.x, minp.y, minp.z);
    Vector3d maxpt(maxp.x, maxp.y, maxp.z);
    return (minpt + maxpt)/2;
}
Vector3d cloudMidpoint(PointCloud<PointXYZ>::ConstPtr cloud1, PointCloud<PointXYZ>::ConstPtr cloud2)
{
    PointXYZ min1, min2, max1, max2;
    getMinMax3D(*cloud1, min1, max1);
    getMinMax3D(*cloud2, min2, max2);
    Vector3d minpt(
            min(min1.x, min2.x),
            min(min1.y, min2.y),
            min(min1.z, min2.z));
    Vector3d maxpt(
            max(max1.x, max2.x),
            max(max1.y, max2.y),
            max(max1.z, max2.z));
    return (minpt + maxpt)/2;
}
#include <vtkVersion.h>
#include <vtkLODActor.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkCellArray.h>
#include <vtkTextProperty.h>
#include <vtkAbstractPropPicker.h>
#include <vtkCamera.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkScalarBarActor.h>
#include <vtkPNGWriter.h>
#include <vtkWindowToImageFilter.h>
#include <vtkRendererCollection.h>
#include <vtkActorCollection.h>
#include <vtkLegendScaleActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkObjectFactory.h>
#include <vtkProperty.h>
#include <vtkPointData.h>
#include <vtkAssemblyPath.h>
#include <vtkAbstractPicker.h>
#include <vtkPointPicker.h>
#include <vtkAreaPicker.h>
using namespace pcl::visualization;
void InteractorStyle::OnChar() {
    // Make sure we ignore the same events we handle in OnKeyDown to avoid calling things twice
    FindPokedRenderer (Interactor->GetEventPosition ()[0], Interactor->GetEventPosition ()[1]);
    std::string key (Interactor->GetKeySym ());
    if (key.find ("XF86ZoomIn") != std::string::npos)
        zoomIn ();
    else if (key.find ("XF86ZoomOut") != std::string::npos)
        zoomOut ();

    bool keymod = false;
    switch (modifier_)
    {
        case INTERACTOR_KB_MOD_ALT:
            {
                keymod = Interactor->GetAltKey ();
                break;
            }
        case INTERACTOR_KB_MOD_CTRL:
            {
                keymod = Interactor->GetControlKey ();
                break;
            }
        case INTERACTOR_KB_MOD_SHIFT:
            {
                keymod = Interactor->GetShiftKey ();
                break;
            }
    }
    Superclass::OnChar ();
}
void InteractorStyle::OnKeyDown() {
    if (!init_)
    {
        pcl::console::print_error ("[PCLVisualizerInteractorStyle] Interactor style not initialized. Please call Initialize () before continuing.\n");
        return;
    }

    if (!rens_)
    {
        pcl::console::print_error ("[PCLVisualizerInteractorStyle] No renderer collection given! Use SetRendererCollection () before continuing.\n");
        return;
    }

    FindPokedRenderer (Interactor->GetEventPosition ()[0], Interactor->GetEventPosition ()[1]);

    if (wif_->GetInput () == NULL)
    {
        wif_->SetInput (Interactor->GetRenderWindow ());
        wif_->Modified ();
        snapshot_writer_->Modified ();
    }

    // Save the initial windows width/height
    if (win_height_ == -1 || win_width_ == -1)
    {
        int *win_size = Interactor->GetRenderWindow ()->GetSize ();
        win_height_ = win_size[0];
        win_width_  = win_size[1];
    }

    // Get the status of special keys (Cltr+Alt+Shift)
    bool shift = Interactor->GetShiftKey   ();
    bool ctrl  = Interactor->GetControlKey ();
    bool alt   = Interactor->GetAltKey ();

    bool keymod = false;
    switch (modifier_)
    {
        case INTERACTOR_KB_MOD_ALT:
            {
                keymod = alt;
                break;
            }
        case INTERACTOR_KB_MOD_CTRL:
            {
                keymod = ctrl;
                break;
            }
        case INTERACTOR_KB_MOD_SHIFT:
            {
                keymod = shift;
                break;
            }
    }
    switch (Interactor->GetKeySym()[0]) {
        case '1':
        case '2':
        case '3':
        case 'q':
        case 'Q':
            break;
        default:
            Superclass::OnKeyDown();
    }
    KeyboardEvent event (true, Interactor->GetKeySym (), Interactor->GetKeyCode (), Interactor->GetAltKey (), Interactor->GetControlKey (), Interactor->GetShiftKey ());
    keyboard_signal_ (event);

    rens_->Render ();
    Interactor->Render ();
}
