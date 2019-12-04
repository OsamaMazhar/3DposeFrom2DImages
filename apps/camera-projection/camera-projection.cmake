# declare_PID_Component(
#   APPLICATION
#   NAME camera-projection
#   DIRECTORY camera-projection
#   RUNTIME_RESOURCES urdf_models app_config pose_files
# )

PID_Component(
APPLICATION
NAME camera-projection
DIRECTORY camera-projection
CXX_STANDARD 14
RUNTIME_RESOURCES urdf_models app_config pose_files
DEPEND opencv-core opencv-highgui opencv-imgproc opencv-videoio opencv-calib3d nlopt
rbdyn/rbdyn-parsers pid-rpath/rpathlib yaml-cpp/libyaml)

# declare_PID_Component_Dependency(
#   COMPONENT camera-projection
#   EXTERNAL nanomsgxx
#   PACKAGE nanomsgxx
# )

declare_PID_Component_Dependency(
  COMPONENT camera-projection
  NATIVE vrep-driver
  PACKAGE api-driver-vrep
)

declare_PID_Component_Dependency(
  COMPONENT camera-projection
  EXTERNAL libflatbuffers
  PACKAGE flatbuffers
)

declare_PID_Component_Dependency(
  COMPONENT camera-projection
  LINKS SHARED  -L/usr/local/cuda/lib64 -L/opt/cuda/lib64 -lm /usr/local/lib/libnnxx.so
  # LINKS SHARED -L/usr/local/cuda/lib64 ${OpenCV_LIBS} /usr/local/lib/libnlopt.so -lm /usr/local/lib/libnnxx.so /home/osama/Programs/flatbuffers/build/libflatbuffers.a
  # LINKS SHARED /home/osama/Program/optim/liboptim.so -larmadillo -L/usr/local/cuda/lib64 ${OpenCV_LIBS}
  # INCLUDE_DIRS /home/osama/Programs/optim/include
  # INCLUDE_DIRS /usr/local/include /home/osama/Programs/flatbuffers/include/
)
