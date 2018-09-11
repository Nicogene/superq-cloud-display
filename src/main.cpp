/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file main.cpp
 * @authors: Fabrizio Bottarel <fabrizio.bottarel@iit.it>
 */

#include <cstdlib>
#include <mutex>
#include <string>
#include <vector>

#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <vtkSmartPointer.h>
#include <vtkCommand.h>
#include <vtkProperty.h>
#include <vtkPolyDataMapper.h>
#include <vtkPointData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkSuperquadric.h>
#include <vtkTransform.h>
#include <vtkSampleFunction.h>
#include <vtkContourFilter.h>
#include <vtkActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleSwitch.h>

#include <vtkVersion.h>


using namespace std;
using namespace yarp::dev;
using namespace yarp::math;
using namespace yarp::os;
using namespace yarp::sig;

class DisplayPointCloud;
class PointCloudProcessor;



class UpdateCommand : public vtkCommand
{
    const bool *closing;

public:

    vtkTypeMacro(UpdateCommand, vtkCommand)


    static UpdateCommand *New()
    {
        return new UpdateCommand;
    }


    UpdateCommand() : closing(nullptr) { }


    void set_closing(const bool &closing)
    {
        this->closing=&closing;
    }


    void Execute(vtkObject *caller, unsigned long vtkNotUsed(eventId),
                 void *vtkNotUsed(callData)) override
    {
        vtkRenderWindowInteractor* iren=static_cast<vtkRenderWindowInteractor*>(caller);
        if (closing!=nullptr)
        {
            if (*closing)
            {
                iren->GetRenderWindow()->Finalize();
                iren->TerminateApp();
                return;
            }
        }

        iren->GetRenderWindow()->SetWindowName("Superquadric viewer");
        iren->Render();
    }
};



class Object
{
protected:
    vtkSmartPointer<vtkPolyDataMapper> vtk_mapper;
    vtkSmartPointer<vtkActor> vtk_actor;

public:

    vtkSmartPointer<vtkActor> &get_actor()
    {
        return vtk_actor;
    }
};



class Points : public Object
{
protected:
    vtkSmartPointer<vtkPoints> vtk_points;
    vtkSmartPointer<vtkUnsignedCharArray> vtk_colors;
    vtkSmartPointer<vtkPolyData> vtk_polydata;
    vtkSmartPointer<vtkVertexGlyphFilter> vtk_glyphFilter;

public:

    Points(const PointCloud<DataXYZ> &points, const int point_size)
    {
        vtk_points=vtkSmartPointer<vtkPoints>::New();
        for (size_t i=0; i<points.size(); i++)
            vtk_points->InsertNextPoint(points(i).x, points(i).y, points(i).z);

        vtk_polydata=vtkSmartPointer<vtkPolyData>::New();
        vtk_polydata->SetPoints(vtk_points);

        vtk_glyphFilter=vtkSmartPointer<vtkVertexGlyphFilter>::New();
        vtk_glyphFilter->SetInputData(vtk_polydata);
        vtk_glyphFilter->Update();

        vtk_mapper=vtkSmartPointer<vtkPolyDataMapper>::New();
        vtk_mapper->SetInputConnection(vtk_glyphFilter->GetOutputPort());

        vtk_actor=vtkSmartPointer<vtkActor>::New();
        vtk_actor->SetMapper(vtk_mapper);
        vtk_actor->GetProperty()->SetPointSize(point_size);
    }


    void set_points(const PointCloud<DataXYZ> &points)
    {
        vtk_points=vtkSmartPointer<vtkPoints>::New();
        for (size_t i=0; i<points.size(); i++)
            vtk_points->InsertNextPoint(points(i).x, points(i).y, points(i).z);

        vtk_polydata->SetPoints(vtk_points);
    }


    bool set_colors(const PointCloud<DataXYZRGBA> &points)
    {
        if (points.size()==vtk_points->GetNumberOfPoints())
        {
            vtk_colors=vtkSmartPointer<vtkUnsignedCharArray>::New();
            vtk_colors->SetNumberOfComponents(3);
            for (size_t i=0; i<points.size(); i++)
            {
                vector<unsigned char> colors = {points(i).r, points(i).g, points(i).b};
                vtk_colors->InsertNextTypedTuple(colors.data());
            }

            vtk_polydata->GetPointData()->SetScalars(vtk_colors);
            return true;
        }
        else
            return false;
    }


    vtkSmartPointer<vtkPolyData> &get_polydata()
    {
        return vtk_polydata;
    }
};



class PointCloudProcessor: public TypedReaderCallback<ImageOf<PixelFloat>>
{
    DisplayPointCloud *displayer{nullptr};
    Property intrinsics;
    using TypedReaderCallback<ImageOf<PixelFloat>>::onRead;
    virtual void onRead(ImageOf<PixelFloat> &image) override;

public:
    PointCloudProcessor(DisplayPointCloud *displayer_, const Property& intrinsics_) : displayer{displayer_},
                                                                                      intrinsics{intrinsics_} { }
};


class DisplayPointCloud : public RFModule
{


public:

    //  set up the constructor
    DisplayPointCloud(): closing(false), PCproc{nullptr} {}

    void refreshPointCloud(const PointCloud<DataXYZ> &points)
    {
       if (points.size() > 0)
       {
           std::lock_guard<std::mutex> lg(mutex);

           //   set the vtk point cloud object with the read data
           vtk_points->set_points(points);
           // To test it later with XYZRGBA
           //vtk_points->set_colors(points);

       }
    }
private:
    std::unique_ptr<PointCloudProcessor> PCproc;

    BufferedPort<ImageOf<PixelFloat>> depthImageInPort;

    string moduleName;

    std::mutex mutex;

    bool closing;

    unique_ptr<Points> vtk_points;

    vtkSmartPointer<vtkRenderer> vtk_renderer;
    vtkSmartPointer<vtkRenderWindow> vtk_renderWindow;
    vtkSmartPointer<vtkRenderWindowInteractor> vtk_renderWindowInteractor;
    vtkSmartPointer<vtkAxesActor> vtk_axes;
    vtkSmartPointer<vtkOrientationMarkerWidget> vtk_widget;
    vtkSmartPointer<vtkCamera> vtk_camera;
    vtkSmartPointer<vtkInteractorStyleSwitch> vtk_style;
    vtkSmartPointer<UpdateCommand> vtk_updateCallback;

    PolyDriver poly;



    bool configure(ResourceFinder &rf) override
    {
        Property intrinsics;

        if (rf.check("name"))
        {
            moduleName = rf.find("name").asString();
        }
        else
        {
            moduleName = "point-cloud-display";
        }

        Property conf;
        conf.put("device","RGBDSensorClient");
        conf.put("localImagePort","/clientRgbPort:i");
        conf.put("localDepthPort","/clientDepthPort:i");
        conf.put("localRpcPort","/clientRpcPort");
        conf.put("remoteImagePort","/depthCamera/rgbImage:o");
        conf.put("remoteDepthPort","/depthCamera/depthImage:o");
        conf.put("remoteRpcPort","/depthCamera/rpc:i");

        if (!poly.open(conf))
        {
            yError()<<"Unable to open RGBDClient polydriver";
            return false;
        }

        yarp::dev::IRGBDSensor* iRgbd{nullptr};
        if (!poly.view(iRgbd))
        {
            yError()<<"Unable to view IRGBD interface";
            return false;
        }

        if (!iRgbd->getDepthIntrinsicParam(intrinsics))
        {
            yError()<<"Unable to get depth intrinsic params";
            return false;
        }

        yDebug()<<"intrinsic param are:"<<intrinsics.toString();

        if (!poly.close())
        {
            yError()<<"Failed to close RGBDClient";
            return false;
        }

        //  open ports
        if (!depthImageInPort.open("/" + moduleName + "/depthImage:i"))
        {
            yError()<<"Unable to open port:"<<depthImageInPort.getName();
            return false;
        }

        PCproc = unique_ptr<PointCloudProcessor>(new PointCloudProcessor(this, intrinsics));
        //  attach callbacks to ports
        depthImageInPort.useCallback(*PCproc);


        //  initialize empty point cloud to display
        PointCloud<DataXYZ> pc;
        pc.clear();
        vtk_points = unique_ptr<Points>(new Points(pc, 3));

        //  set up renderer window
        vtk_renderer = vtkSmartPointer<vtkRenderer>::New();
        vtk_renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
        vtk_renderWindow->SetSize(600,600);
        vtk_renderWindow->AddRenderer(vtk_renderer);
        vtk_renderWindowInteractor=vtkSmartPointer<vtkRenderWindowInteractor>::New();
        vtk_renderWindowInteractor->SetRenderWindow(vtk_renderWindow);

        //  set up actors for the superquadric and point cloud
        vtk_renderer->AddActor(vtk_points->get_actor());
        vtk_renderer->SetBackground(0.1,0.2,0.2);

        //  set up axes widget
        vtk_axes = vtkSmartPointer<vtkAxesActor>::New();
        vtk_widget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
        vtk_widget->SetOutlineColor(0.9300,0.5700,0.1300);
        vtk_widget->SetOrientationMarker(vtk_axes);
        vtk_widget->SetInteractor(vtk_renderWindowInteractor);
        vtk_widget->SetViewport(0.0,0.0,0.2,0.2);
        vtk_widget->SetEnabled(1);
        vtk_widget->InteractiveOn();

        //  set up the camera position according to the point cloud distribution
        vector<double> pc_bounds(6), pc_centroid(3);
        vtk_points->get_polydata()->GetBounds(pc_bounds.data());
        for (size_t i=0; i<pc_centroid.size(); i++)
            pc_centroid[i] = 0.5*(pc_bounds[i<<1]+pc_bounds[(i<<1)+1]);

        vtk_camera = vtkSmartPointer<vtkCamera>::New();
        vtk_camera->SetPosition(pc_centroid[0]+1.0, pc_centroid[1], pc_centroid[2]+0.5);
        vtk_camera->SetViewUp(0.0, 0.0, 1.0);
        vtk_renderer->SetActiveCamera(vtk_camera);

        vtk_style=vtkSmartPointer<vtkInteractorStyleSwitch>::New();
        vtk_style->SetCurrentStyleToTrackballCamera();
        vtk_renderWindowInteractor->SetInteractorStyle(vtk_style);

        vtk_renderWindowInteractor->Initialize();
        vtk_renderWindowInteractor->CreateRepeatingTimer(10);

        //  set up the visualizer refresh callback
        vtk_updateCallback = vtkSmartPointer<UpdateCommand>::New();
        vtk_updateCallback->set_closing(closing);
        vtk_renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, vtk_updateCallback);
        vtk_renderWindowInteractor->Start();

        return true;

    }


    bool updateModule() override
    {
        //  do something during update?

        return true;

    }


    bool interruptModule() override
    {

        depthImageInPort.interrupt();
        closing = true;

        return true;

    }


    bool close() override
    {
        if (!depthImageInPort.isClosed())
            depthImageInPort.close();
        return true;

    }

};

void PointCloudProcessor::onRead(ImageOf<PixelFloat> &image)
{
    //  define what to do when the callback is triggered
    //  i.e. when a point cloud comes in
    // yDebug()<<"Reading PC...";
    displayer->refreshPointCloud(yarp::math::depthToPC(image, intrinsics));
}



int main(int argc, char *argv[])
{

    Network yarp;
    ResourceFinder rf;
    rf.configure(argc, argv);

    if (!yarp.checkNetwork())
    {
        yError() << "YARP network not detected. Check nameserver";
        return EXIT_FAILURE;
    }

    DisplayPointCloud disp;

    return disp.runModule(rf);
}
