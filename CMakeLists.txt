CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
set(WORKSPACE_DIR ${CMAKE_SOURCE_DIR}/../.. CACHE PATH "root of the packages workspace directory")
list(APPEND CMAKE_MODULE_PATH ${WORKSPACE_DIR}/share/cmake/system) # using generic scripts/modules of the workspace
include(Package_Definition NO_POLICY_SCOPE)
set(CMAKE_REQUIRED_FLAGS -std=c++14)

PROJECT(pose-estimation)

declare_PID_Package(
			AUTHOR      osama
			YEAR        2018
			LICENSE     CeCILL
			ADDRESS		git@gite.lirmm.fr:mazhar/pose-estimation.git
			DESCRIPTION TODO: input a short description of package pose-estimation utility here
		)

# set_PID_Package_Version(0.7.0)

PID_Dependency(rbdyn VERSION 1.2.0)
PID_Dependency(pid-rpath VERSION 2.1.0)
PID_Dependency(yaml-cpp VERSION 0.6.2)
PID_Dependency(api-driver-vrep VERSION 1.0.0)
PID_Dependency(nanomsgxx VERSION 0.3.0)
PID_Dependency(flatbuffers VERSION 1.9.1)
PID_Dependency(opencv VERSION 3.4.0)
PID_Dependency(nlopt VERSION 2.5.0)

#now finding packages
# find_package(OpenCV REQUIRED)

build_PID_Package()
