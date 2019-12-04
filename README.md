
Overview
=========

TODO: input a short description of package pose-estimation utility here



The license that applies to the whole package content is **CeCILL**. Please look at the license.txt file at the root of this repository.

Installation and Usage
=======================

The detailed procedures for installing the pose-estimation package and for using its components is based on the [PID](http://pid.lirmm.net/pid-framework/pages/install.html) build and deployment system called PID. Just follow and read the links to understand how to install, use and call its API and/or applications.

For a quick installation:

## Installing the project into an existing PID workspace

To get last version :
 ```
cd <path to pid workspace>/pid
make deploy package=pose-estimation
```

To get a specific version of the package :
 ```
cd <path to pid workspace>/pid
make deploy package=pose-estimation version=<version number>
```

## Standalone install
 ```
git clone git@gite.lirmm.fr:mazhar/pose-estimation.git
cd pose-estimation
```

Then run the adequate install script depending on your system. For instance on linux:
```
sh share/install/standalone_install.sh
```

The pkg-config tool can be used to get all links and compilation flags for the libraries defined inthe project. To let pkg-config know these libraries, read the last output of the install_script and apply the given command. It consists in setting the PKG_CONFIG_PATH, for instance on linux do:
```
export PKG_CONFIG_PATH=<path to pose-estimation>/binaries/pid-workspace/share/pkgconfig:$PKG_CONFIG_PATH
```

Then, to get compilation flags run:

```
pkg-config --static --cflags pose-estimation_<name of library>
```

To get linker flags run:

```
pkg-config --static --libs pose-estimation_<name of library>
```


About authors
=====================

pose-estimation has been developped by following authors: 
+ osama ()

Please contact osama -  for more information or questions.



