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


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


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
    Points(const vector<Vector> &points, const int point_size)
    {
        vtk_points=vtkSmartPointer<vtkPoints>::New();
        for (size_t i=0; i<points.size(); i++)
            vtk_points->InsertNextPoint(points[i][0],points[i][1],points[i][2]);

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
    void set_points(const vector<Vector> &points)
    {
        vtk_points=vtkSmartPointer<vtkPoints>::New();
        for (size_t i=0; i<points.size(); i++)
            vtk_points->InsertNextPoint(points[i][0],points[i][1],points[i][2]);

        vtk_polydata->SetPoints(vtk_points);
    }

    /****************************************************************/
    bool set_colors(const vector<vector<unsigned char>> &colors)
    {
        if (colors.size()==vtk_points->GetNumberOfPoints())
        {
            vtk_colors=vtkSmartPointer<vtkUnsignedCharArray>::New();
            vtk_colors->SetNumberOfComponents(3);
            for (size_t i=0; i<colors.size(); i++)
                vtk_colors->InsertNextTypedTuple(colors[i].data());

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
     * epsilon_1 is e (roundedness/squareness east/west - theta roundedness)
     * epsilon_2 is n (roundedness/sqareness north/south - phi roundedness)
     * scale is sx, sy, sz
     * center is cx, cy, cz
     * rotation parameters?
     */


public:
    /****************************************************************/
    Superquadric(const Vector &r)
    {

        //  Default has radius of 0.5, toroidal off, center at 0.0, scale (1,1,1), size 0.5, phi roundness 1.0, and theta roundness 0.0.
        vtk_superquadric=vtkSmartPointer<vtkSuperquadric>::New();

        //  set the superquadric default values

        vtk_sample=vtkSmartPointer<vtkSampleFunction>::New();
        vtk_sample->SetSampleDimensions(50,50,50);
        vtk_sample->SetImplicitFunction(vtk_superquadric);
        vtk_sample->SetModelBounds(-r[4],r[4],-r[5],r[5],-r[6],r[6]);

        vtk_contours=vtkSmartPointer<vtkContourFilter>::New();
        vtk_contours->SetInputConnection(vtk_sample->GetOutputPort());
        vtk_contours->GenerateValues(1.0,1.0,1.0);

        vtk_mapper=vtkSmartPointer<vtkPolyDataMapper>::New();
        vtk_mapper->SetInputConnection(vtk_contours->GetOutputPort());
        vtk_mapper->SetScalarRange(0.0,1.2);

        vtk_actor=vtkSmartPointer<vtkActor>::New();
        vtk_actor->SetMapper(vtk_mapper);
        vtk_actor->GetProperty()->SetOpacity(0.25);
    }

    /****************************************************************/
    void set_parameters(const Vector &r)
    {
        //  set coefficients of the superquadric

        //  translate and set the pose of the superquadric

    }
};

/****************************************************************/


class DisplaySuperQ : public RFModule
{

    class PointCloudProcessor: public TypedReaderCallback<Matrix>
    {
        using TypedReaderCallback<Matrix>::onRead;
        virtual void onRead(Matrix &pointCloud)
        {
            //  define what to do when the callback is triggered
            //  i.e. when a point cloud comes in
        }
    };

    class SuperquadricProcessor: public TypedReaderCallback<Property>
    {
        using TypedReaderCallback<Property>::onRead;
        virtual void onRead(Property &superQ)
        {
            //  define what to do when the callback is triggered
            //  i.e. when a superquadric comes in
        }
    };


    PointCloudProcessor PCproc;
    SuperquadricProcessor SQproc;

    BufferedPort<Matrix> pointCloudInPort;
    BufferedPort<Property> superqInPort;

    string moduleName;


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
        superqInPort.useCallback(SQproc);

        //  open ports
        pointCloudInPort.open("/" + moduleName + "/pointCloud:i");
        superqInPort.open("/" + moduleName + "/superquadric:i");

    }

    bool updateModule() override
    {
        //  do something during update?

        return true;

    }

    bool interruptModule() override
    {
        superqInPort.interrupt();
        pointCloudInPort.interrupt();

        return true;

    }

    bool close() override
    {
        if (!superqInPort.isClosed())
            superqInPort.close();
        if (!pointCloudInPort.isClosed())
            pointCloudInPort.close();

        return true;

    }
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

    return EXIT_SUCCESS;
}
