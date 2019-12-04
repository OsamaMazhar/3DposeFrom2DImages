# declare_PID_Component(
#   APPLICATION
#   NAME create-projection
#   DIRECTORY create-projection
#   RUNTIME_RESOURCES urdf_models app_config pose_files
# )

PID_Component(
APPLICATION
NAME create-projection
DIRECTORY create-projection
RUNTIME_RESOURCES urdf_models app_config pose_files
DEPEND opencv-core opencv-highgui opencv-imgproc opencv-calib3d nlopt)

declare_PID_Component_Dependency(
  COMPONENT create-projection
  NATIVE rbdyn-parsers
  PACKAGE rbdyn
)

declare_PID_Component_Dependency(
  COMPONENT create-projection
  NATIVE rpathlib
  PACKAGE pid-rpath
)

declare_PID_Component_Dependency(
  COMPONENT create-projection
  EXTERNAL libyaml
  PACKAGE yaml-cpp
)

declare_PID_Component_Dependency(
  COMPONENT create-projection
  NATIVE vrep-driver
  PACKAGE api-driver-vrep
)

# declare_PID_Component_Dependency(
#   COMPONENT create-projection
#   EXTERNAL lib/libopencv_core
#   PACKAGE opencv
# )

declare_PID_Component_Dependency(
  COMPONENT create-projection
  LINKS SHARED -L/usr/local/cuda/lib64 -L/opt/cuda/lib64
  # LINKS SHARED /home/osama/Program/optim/liboptim.so -larmadillo -L/usr/local/cuda/lib64 ${OpenCV_LIBS}
  # INCLUDE_DIRS /home/osama/Programs/optim/include
  # INCLUDE_DIRS /usr/local/include
)
