# superq-cloud-display
Display the superquadric computed by [superquadric-model](https://github.com/robotology/superquadric-model) together with those computed by [find-superquadric](https://github.com/pattacini/find-superquadric) in order to visually evaluate the fit with the corresponding point cloud in a 3D space.

### Dependencies
- [Yarp](https://github.com/robotology/yarp)
- [iCub](https://github.com/robotology/icub-main)
- [VTK](https://github.com/Kitware/VTK)

### Usage
The module is supposed to be used as a 3D visualization tool for the output of [superquadric-model](https://github.com/robotology/superquadric-model)(https://github.com/robotology/superquadric-model) and [find-superquadric](https://github.com/pattacini/find-superquadric). Once run, the following YARP ports are opened:
- `/superq-cloud-display/pointCloud:i` is meant to receive the point cloud stream from an acquisition module, e.g. [point-cloud-read](https://github.com/fbottarel/point-cloud-read)
- `superq-cloud-display/superquadricFiniteDiff:i` is meant to receive the parameters of superquadrics computed by [superquadric-model](https://github.com/robotology/superquadric-model)(https://github.com/robotology/superquadric-model)
- `superq-cloud-display/superquadricAnalyticalGrad:i` is meant to receive the parameters of superquadrics computed by [find-superquadric](https://github.com/pattacini/find-superquadric)

The module spawns a 3D rendering window containing the point cloud and superquadrics. The key bindings to move the camera are explained [here](https://www.vtk.org/doc/nightly/html/classvtkInteractorStyleImage.html) in the Detailed Description section.

### Notes
This will work as a helper to any module, not only `superquadric-model`, `find-superquadric` and `point-cloud-read`. However, in order to achieve this, the following constraints must hold
- `/superq-cloud-display/pointCloud:i` is a `yarp::os::BufferedPort<Matrix>`, where points are stored as lines in the format
 ```
  x0 y0 z0 [r0 g0 b0]
  x1 y1 z1 [r1 g1 b1]
  ...
  ```
- `/superq-cloud-display/superquadricAnalyticalGrad:i` and `/superq-cloud-display/superquadricFiniteDiff:i` must receive messages in the form of `yarp::sig::Bottle` or `yarp::sig::Property` containing the parameters as nested lists in the format `(dimensions (x0 x1 x2)) (exponents (x3 x4)) (center (x5 x6 x7)) (orientation (x8 x9 x10 x11))`
