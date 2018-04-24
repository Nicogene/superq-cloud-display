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
#include <string>
#include <vector>

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
using namespace yarp::os;
using namespace yarp::sig;

/****************************************************************/

class UpdateCommand : public vtkCommand
{
    const bool *closing;

public:
    /****************************************************************/
    vtkTypeMacro(UpdateCommand, vtkCommand);

    /****************************************************************/
    static UpdateCommand *New()
    {
        return new UpdateCommand;
    }

    /****************************************************************/
    UpdateCommand() : closing(nullptr) { }

    /****************************************************************/
    void set_closing(const bool &closing)
    {
        this->closing=&closing;
    }

    /****************************************************************/
    void Execute(vtkObject *caller, unsigned long vtkNotUsed(eventId),
                 void *vtkNotUsed(callData))
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

/****************************************************************/

class Object
{
protected:
    vtkSmartPointer<vtkPolyDataMapper> vtk_mapper;
    vtkSmartPointer<vtkActor> vtk_actor;

public:
    /****************************************************************/
    vtkSmartPointer<vtkActor> &get_actor()
    {
        return vtk_actor;
    }
};

/****************************************************************/

class Points : public Object
{
protected:
    vtkSmartPointer<vtkPoints> vtk_points;
    vtkSmartPointer<vtkUnsignedCharArray> vtk_colors;
    vtkSmartPointer<vtkPolyData> vtk_polydata;
    vtkSmartPointer<vtkVertexGlyphFilter> vtk_glyphFilter;

public:
    /****************************************************************/
    Points(const PointCloud<DataXYZRGBA> &points, const int point_size)
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

    /****************************************************************/
    void set_points(const PointCloud<DataXYZRGBA> &points)
    {
        vtk_points=vtkSmartPointer<vtkPoints>::New();
        for (size_t i=0; i<points.size(); i++)
            vtk_points->InsertNextPoint(points(i).x, points(i).y, points(i).z);

        vtk_polydata->SetPoints(vtk_points);
    }

    /****************************************************************/
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

    /****************************************************************/
    vtkSmartPointer<vtkPolyData> &get_polydata()
    {
        return vtk_polydata;
    }
};

/****************************************************************/

class Superquadric : public Object
{
protected:
    vtkSmartPointer<vtkSuperquadric> vtk_superquadric;
    vtkSmartPointer<vtkSampleFunction> vtk_sample;
    vtkSmartPointer<vtkContourFilter> vtk_contours;
    vtkSmartPointer<vtkTransform> vtk_transform;

    /*
     * Parameters:
     * epsilon_2 is e (roundedness/squareness east/west - theta roundedness)
     * epsilon_1 is n (roundedness/sqareness north/south - phi roundedness)
     * scale is sx, sy, sz
     * center is cx, cy, cz
     * rotation parameters?
     */


public:
    /****************************************************************/
    Superquadric(const Vector &r, const double color)
    {

        //  Default has radius of 0.5, toroidal off, center at 0.0, scale (1,1,1), size 0.5, phi roundness 1.0, and theta roundness 0.0.
        vtk_superquadric=vtkSmartPointer<vtkSuperquadric>::New();

        vtk_superquadric->SetSize(1.0);

        vtk_sample=vtkSmartPointer<vtkSampleFunction>::New();
        vtk_sample->SetSampleDimensions(50,50,50);
        vtk_sample->SetImplicitFunction(vtk_superquadric);
        vtk_sample->SetModelBounds(0, 0, 0, 0, 0, 0);

        vtk_contours=vtkSmartPointer<vtkContourFilter>::New();
        vtk_contours->SetInputConnection(vtk_sample->GetOutputPort());
        vtk_contours->GenerateValues(1,1.0,1.0);

        vtk_mapper=vtkSmartPointer<vtkPolyDataMapper>::New();
        vtk_mapper->SetInputConnection(vtk_contours->GetOutputPort());
        vtk_mapper->SetScalarRange(0.0,color);

        vtk_actor=vtkSmartPointer<vtkActor>::New();
        vtk_actor->SetMapper(vtk_mapper);
        vtk_actor->GetProperty()->SetOpacity(0.25);

        vtk_transform = vtkSmartPointer<vtkTransform>::New();
        vtk_transform->Identity();
        vtk_actor->SetUserTransform(vtk_transform);
    }

    /****************************************************************/
    void set_parameters(const Vector &r)
    {
        //  set coefficients of the superquadric
        //  (dimensions (x0 x1 x2)) (exponents (x3 x4)) (center (x5 x6 x7)) (orientation (x8 x9 x10 x11))
        //  suppose x8 as angle
        vtk_superquadric->SetScale(r[0], r[1], r[2]);
        vtk_superquadric->SetPhiRoundness(r[3]);
        vtk_superquadric->SetThetaRoundness(r[4]);


        yInfo() << "Phi roundedness:" << r[3];
        yInfo() << "Theta roundedness:" << r[4];

        vtk_sample->SetModelBounds(-2*r[0], 2*r[0], -2*r[1], 2*r[1], -2*r[2], 2*r[2]);

        //  translate and set the pose of the superquadric
        vtk_superquadric->SetCenter(0.0, 0.0, 0.0);
        vtk_superquadric->SetToroidal(0);
        vtk_transform->Identity();
        vtk_transform->Translate(r[5], r[6], r[7]);
        vtk_transform->RotateWXYZ(r[8], r[9], r[10], r[11]);

    }
};

/****************************************************************/

class DisplaySuperQ : public RFModule
{
    enum class SuperquadricType
    {
        //  we need to handle different superquadrics at once
        ANALYTICAL_GRAD_SQ,
        FINITE_DIFF_SQ
    };

    class PointCloudProcessor: public TypedReaderCallback<PointCloud<DataXYZRGBA>>
    {
        DisplaySuperQ *displayer;
        using TypedReaderCallback<PointCloud<DataXYZRGBA>>::onRead;
        virtual void onRead(PointCloud<DataXYZRGBA> &pointCloud)
        {
            //  define what to do when the callback is triggered
            //  i.e. when a point cloud comes in
            displayer->refreshPointCloud(pointCloud);
            //  TODO: a point cloud that gets read here should trigger the acquisition of
            //  both superquadrics with rpcs. We don't have the rpc from superquadric-model
            //  fixed yet so we only trigger the acquisition from find-superquadric
            //  via displayer->refreshSuperquadric
            //  prepare cmd, reply for find-superquadric
            //  write on rpc port
            //  convert syntax of response into a Property
            //  call refreshSuperquadric
            Bottle sq_reply;
            sq_reply.clear();

            displayer->superq2RPC.write(pointCloud, sq_reply);
            Property superquadric_params;

            Bottle bottle;
            Bottle &b1=bottle.addList();
            b1.addDouble(sq_reply.get(4).asDouble());
            b1.addDouble(sq_reply.get(5).asDouble());
            b1.addDouble(sq_reply.get(6).asDouble());
            superquadric_params.put("dimensions", bottle.get(0));

            Bottle &b2=bottle.addList();
            b2.addDouble(sq_reply.get(8).asDouble());
            b2.addDouble(sq_reply.get(9).asDouble());
            superquadric_params.put("exponents", bottle.get(1));

            Bottle &b3=bottle.addList();
            b3.addDouble(sq_reply.get(0).asDouble());
            b3.addDouble(sq_reply.get(1).asDouble());
            b3.addDouble(sq_reply.get(2).asDouble());
            superquadric_params.put("center", bottle.get(2));

            Bottle &b4=bottle.addList();
            b4.addDouble(sq_reply.get(3).asDouble()); b4.addDouble(0.0); b4.addDouble(0.0); b4.addDouble(1.0);
            superquadric_params.put("orientation", bottle.get(3));

            displayer->refreshSuperquadric(superquadric_params, SuperquadricType::ANALYTICAL_GRAD_SQ);

        }
    public:
        PointCloudProcessor(DisplaySuperQ *displayer_) : displayer(displayer_) { }
    };

    class SuperquadricProcessor: public TypedReaderCallback<Property>
    {
        DisplaySuperQ *displayer;
        SuperquadricType sq_type;
        using TypedReaderCallback<Property>::onRead;
        virtual void onRead(Property &superQ)
        {
            //  define what to do when the callback is triggered
            //  i.e. when a superquadric comes in
            displayer->refreshSuperquadric(superQ, sq_type);
        }
    public:
        SuperquadricProcessor(DisplaySuperQ *displayer_, SuperquadricType type) : displayer(displayer_), sq_type(type) { }
    };

    PointCloudProcessor PCproc;
    SuperquadricProcessor SQprocFD, SQprocAG;   //  FD = finite differences, AG = analytical gradient

    BufferedPort<PointCloud<DataXYZRGBA>> pointCloudInPort;
    BufferedPort<Property> superq1InPort;
    BufferedPort<Property> superq2InPort;

    RpcClient superq1RPC;        //  not needed for now!
    RpcClient superq2RPC;

    string moduleName;

    Mutex mutex;

    bool closing;

    unique_ptr<Points> vtk_points;
    unique_ptr<Superquadric> vtk_superquadric_fin_diff;
    unique_ptr<Superquadric> vtk_superquadric_an_grad;

    vtkSmartPointer<vtkRenderer> vtk_renderer;
    vtkSmartPointer<vtkRenderWindow> vtk_renderWindow;
    vtkSmartPointer<vtkRenderWindowInteractor> vtk_renderWindowInteractor;
    vtkSmartPointer<vtkAxesActor> vtk_axes;
    vtkSmartPointer<vtkOrientationMarkerWidget> vtk_widget;
    vtkSmartPointer<vtkCamera> vtk_camera;
    vtkSmartPointer<vtkInteractorStyleSwitch> vtk_style;
    vtkSmartPointer<UpdateCommand> vtk_updateCallback;


    bool configure(ResourceFinder &rf) override
    {

        if (rf.check("name"))
        {
            moduleName = rf.find("name").asString();
        }
        else
        {
            moduleName = "superq-cloud-display";
        }

        //  attach callbacks to ports
        pointCloudInPort.useCallback(PCproc);
        superq1InPort.useCallback(SQprocFD);
        superq2InPort.useCallback(SQprocAG);

        //  open ports
        pointCloudInPort.open("/" + moduleName + "/pointCloud:i");
        superq1InPort.open("/" + moduleName + "/superquadricFiniteDiff:i");
        superq2InPort.open("/" + moduleName + "/superquadricAnalyticalGrad:i");
        //superq1RPC.open("/" + moduleName + "/superquadricFiniteDiff:rpc");
        superq2RPC.open("/" + moduleName + "/superquadricAnalyticalGrad:rpc");

        //  initialize empty point cloud to display
        PointCloud<DataXYZRGBA> pc;
        pc.clear();
        vtk_points = unique_ptr<Points>(new Points(pc, 3));

        //  initialize superquadric
        Vector r(11, 0.0);
        vtk_superquadric_fin_diff = unique_ptr<Superquadric>(new Superquadric(r, 2.0));
        vtk_superquadric_an_grad = unique_ptr<Superquadric>(new Superquadric(r, 1.2));

        //  set up renderer window
        vtk_renderer = vtkSmartPointer<vtkRenderer>::New();
        vtk_renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
        vtk_renderWindow->SetSize(600,600);
        vtk_renderWindow->AddRenderer(vtk_renderer);
        vtk_renderWindowInteractor=vtkSmartPointer<vtkRenderWindowInteractor>::New();
        vtk_renderWindowInteractor->SetRenderWindow(vtk_renderWindow);

        //  set up actors for the superquadric and point cloud
        vtk_renderer->AddActor(vtk_points->get_actor());
        vtk_renderer->AddActor(vtk_superquadric_fin_diff->get_actor());
        vtk_renderer->AddActor(vtk_superquadric_an_grad->get_actor());
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

    /****************************************************************/
    bool updateModule() override
    {
        //  do something during update?

        return true;

    }

    /****************************************************************/
    bool interruptModule() override
    {
        superq1InPort.interrupt();
        superq2InPort.interrupt();
        //superq1RPC.interrupt();
        superq2RPC.interrupt();
        pointCloudInPort.interrupt();
        closing = true;

        return true;

    }

    /****************************************************************/
    bool close() override
    {
        if (!superq1InPort.isClosed())
            superq1InPort.close();
//        if (!superq1RPC.isClosed())
//            superq1RPC.close();
        superq2RPC.close();
        if (!superq2InPort.isClosed())
            superq2InPort.close();
        if (!pointCloudInPort.isClosed())
            pointCloudInPort.close();

        return true;

    }

    /****************************************************************/
    void refreshPointCloud(const PointCloud<DataXYZRGBA> &points)
    {
       if (points.size() > 0)
       {
           LockGuard lg(mutex);

           //   set the vtk point cloud object with the read data
           vtk_points->set_points(points);
           vtk_points->set_colors(points);

       }
    }

    /****************************************************************/
    void refreshSuperquadric(const Property &superq, const SuperquadricType sq_type)
    {
        //  the incoming message has the following syntax
        //  (dimensions (x0 x1 x2)) (exponents (x3 x4)) (center (x5 x6 x7)) (orientation (x8 x9 x10 x11))

        //  pass the parameters as a single vector
        Vector params;

        Bottle *superq_dimension = superq.find("dimensions").asList();
        if (!superq_dimension->isNull())
            for (int i=0; i<superq_dimension->size(); i++)
                params.push_back(superq_dimension->get(i).asDouble());

        Bottle *superq_exponents = superq.find("exponents").asList();
        if (!superq_exponents->isNull())
            for (int i=0; i<superq_exponents->size(); i++)
                params.push_back(superq_exponents->get(i).asDouble());

        Bottle *superq_center = superq.find("center").asList();
        if (!superq_center->isNull())
            for (int i=0; i<superq_center->size(); i++)
                params.push_back(superq_center->get(i).asDouble());

        Bottle *superq_orientation = superq.find("orientation").asList();
        if (!superq_orientation->isNull())
            for (int i=0; i<superq_orientation->size(); i++)
                params.push_back(superq_orientation->get(i).asDouble());

        //  check whether we have a sufficient number of parameters
        if (params.size() == 12)
            switch (sq_type)
            {
            case SuperquadricType::FINITE_DIFF_SQ:
                vtk_superquadric_fin_diff->set_parameters(params);
                break;
            case SuperquadricType::ANALYTICAL_GRAD_SQ:
                vtk_superquadric_an_grad->set_parameters(params);
                break;
            }
        else
            yError() << "Invalid superquadric";

    }

    /****************************************************************/
public:

    //  set up the constructor
    DisplaySuperQ(): closing(false), PCproc(this), SQprocAG(this, SuperquadricType::ANALYTICAL_GRAD_SQ),
        SQprocFD(this, SuperquadricType::FINITE_DIFF_SQ) {}
};

/****************************************************************/

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

    DisplaySuperQ disp;

    return disp.runModule(rf);
}
