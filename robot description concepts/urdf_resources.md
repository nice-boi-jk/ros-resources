## URDF resources
- [urdf ros wiki](http://wiki.ros.org/urdf)
- [xml tags and specifications for urdf](http://wiki.ros.org/urdf/XML)
- [using urdf with gazebo](https://classic.gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros)
## Random
#### some tags i found in a document
- found this in /opt/ros/melodic/include/urdf/urdfdom_compatibility.h
- Think all the defining tags used in a urdf mainly for links and joints is listed here. Do with it what you will.
- Also, i notice that the <plugin> tag is missing, maybe others are missing as well.
```
namespace urdf
{
// These shared_ptrs were added to urdfdom in 0.4.0,
// so if urdfdom == 0.4 this is duplicate work
URDF_TYPEDEF_CLASS_POINTER(Box);
URDF_TYPEDEF_CLASS_POINTER(Collision);
URDF_TYPEDEF_CLASS_POINTER(Cylinder);
URDF_TYPEDEF_CLASS_POINTER(Geometry);
URDF_TYPEDEF_CLASS_POINTER(Inertial);

URDF_TYPEDEF_CLASS_POINTER(Joint);
URDF_TYPEDEF_CLASS_POINTER(JointCalibration);
URDF_TYPEDEF_CLASS_POINTER(JointDynamics);
URDF_TYPEDEF_CLASS_POINTER(JointLimits);
URDF_TYPEDEF_CLASS_POINTER(JointMimic);
URDF_TYPEDEF_CLASS_POINTER(JointSafety);

URDF_TYPEDEF_CLASS_POINTER(Link);
URDF_TYPEDEF_CLASS_POINTER(Material);
URDF_TYPEDEF_CLASS_POINTER(Mesh);
URDF_TYPEDEF_CLASS_POINTER(Sphere);
URDF_TYPEDEF_CLASS_POINTER(Visual);

URDF_TYPEDEF_CLASS_POINTER(ModelInterface);

// ModelSharedPtr is the only one that needs to be defined for urdfdom 0.4
URDF_TYPEDEF_CLASS_POINTER(Model);
}  // namespace urdf
```
