# Refactored_ORB_SLAM2

This is a refactoring of the ORB_SLAM2 repository. It uses up-to-date cmake, up-to-date DBoW2 and g2o libraries, supports all static and all dynamic libraries (for debugging) and 2011 era C++ for sleep, threads, namespaces, etc.

It has been succesfully built on Ubunutu 18.04. Ubuntu 20.04, Windows 10 (natively), Intel Mac, M1 Mac.

## Prerequisites

This version depends on a few system libraries:

1. eigen3
2. boost
3. OpenCV (either 3.x or 4.x)
4. Suite sparse

The code comes shipped with matched versions of DLib and DBoW2 (for the bag of words for data association), g2o (both front and backend optimization) and pangolin (GUI).

The build instructions are deliberately designed to be similar on all supported operating systems.

## Linux build instructions


Clone this repository.

You should be able to build by running:

`./Build.sh`

to build a debug version. To build a release version, type:

`./Build.sh Release`

If you want to avoid typing `./Install/bin` everywhere, run this command from the command line:

`set PATH=$PATH:$PWD/Install/bin`


## Mac (Intel and Apple Silicon) build instructions.

We use homebrew and build using the XCode command line tools. Please ensure that both have been installed.

Install the support libraries:

brew install boost eigen3 suitesparse opencv

Clone this repository.

You should be able to build by running:

`./Build.sh`

to build a debug version. To build a release version, type:

`./Build.sh Release`

If you want to avoid typing `./Install/bin` everywhere, run this command from the command line:

`set PATH=$PATH:$PWD/Install/bin`

## Windows 10 build

Windows 10 is a more challenging OS to build on because it doesn't have hard standards for how to lay out build and install. We use vcpkg.

PLEASE MAKE SURE TO CLEAR THE CMAKE PACKAGE REGISTRY BEFORE YOU TRY TO BUILD. WE HAVE ENCOUNTERED A LOT OF PROBLEMS WITH THE PACKAGE REGISTRY BEING USED TO LINK TO THE WRONG VERSIONS OF LIBRARIES, LEADING TO A LOT OF CONFUSION AND FRUSTRATION.

First, install vcpkg

Second,

You should be able to build by running:

`Build.bat`

to build a debug version. To build a release version, type:

`Build.bat Release`

This will launch four build jobs in parallel. If your machine can take it, you can task more cpus by changing the value passed to maxcpucount


##### Difference on execution against original  ORB_SLAM2:

1.  All executables are installed in `./Install/bin` and a suffix is used to denote debug builds.

   For instance, if you want to run mono_kitti with the default Build.sh file setting, the executable is located at `./Install/bin/mono_kitti_d`

2. The standard ORB vocabulary is loaded by default and does not have to be specified on the command line. (It is installed in `./Install/var/lib/orbslam2` subdirectory). The system will try to load the binary version of the vocabulary. If it is not able to, it will load the text version, convert it, and save the binary version to the same directory. This speeds up start up times from several seconds to less than 0.5s.

3. The standard settings files are installed in `./Install/etc/orbslam2/Monocular`. You do not need to specify the directory. Therefore, a example command for running KITTI00 (using a debug build) is:

   `./Install/bin/mono_kitti_d KITTI00-02.yaml ${your_kitti_dataset_folder}/sequences/00`
   
 The release build would be:
 
   `./Install/bin/mono_kitti KITTI00-02.yaml ${your_kitti_dataset_folder}/sequences/00`
   
If you have set the `PATH` variable as specified above, you can change it to:

   `mono_kitti KITTI00-02.yaml ${your_kitti_dataset_folder}/sequences/00`

  
On linux it is highly recommend you get used to using valgrind. See https://www.valgrind.org/docs/manual/quick-start.html

##### Other examples

Suppose we wish to run `mono_tum` on the Freiburg fr1/xyz dataset. Download the data set from the repository directory (https://vision.in.tum.de/data/datasets/rgbd-dataset/download) and uncompress it to, say your `~/Downloads` directory. This should produce a directory called  `rgbd_dataset_freiburg1_xyz`.

You can then run the code using:

`./Install/bin/mono_tum TUM1.yaml ~/Downloads/rgbd_dataset_freiburg1_xyz`   
   
 Or, if you did the trick with setting `PATH`, this can be simplified to:
   
`./mono_tum TUM1.yaml ~/Downloads/rgbd_dataset_freiburg1_xyz`


## Windows 10 build - A step by step instruction:

List of things to install:

1. Install visual studio 2019, git and cmake
   1. If you have installed Unity before, then it's very likely that you already have visual studio installed.
   2. For cmake:
      1. https://cmake.org/download/
      2. add CMake to environmental PATH in installation selection. 
      3. ![ select "add CMake to environmental PATH"](Doc\cmake.png " add CMake to environmental PATH")

   1. restart your machine.

2. install vcpkg and configure environmental variable 
   1. https://vcpkg.io/en/getting-started.html
   2. (I personally placed vcpkg in `C:\src\vcpkg` )
3. 
   
4. use vcpkg to install other dependencies
   1. vcpkg

Make sure you are at vcpkg source directory: something like this:
 `C:\src> .\vcpkg\vcpkg.exe install opencv`
 depends on your internet speed and how powerful your CPU is, each of the following may take a while. The elapsed time below are from my laptop, it's relatively old so yours should be faster. Please have your laptop plugged in.
   2. Opencv  (Total elapsed time: 26.29 min)
      1.`vcpkg.exe install opencv`
   3. boost (Total elapsed time: 33.99 min)
      1. `vcpkg.exe install boost`
   4. Eigen3 (39.73 s)
      1. `vcpkg.exe install eigen3`
1. Configure environmental variables:
   1. msbuild
      1. https://debajmecrm.com/how-to-resolve-msbuild-is-not-recognized-as-internal-or-external-command-error-in-visual-studio-code/
      2. !["add msbuild path to PATH"](Doc\PATH_vs.png " add msbuild to environmental PATH")
   2. VCPKG_ROOT
      1. It should be where you placed your vcpkg package, for my case it's  C:\src\
      2. !["add VCPKG_ROOT path to environmental variable"](Doc\VCPKG_ROOT.png " add VCPKG_ROOT to environmental variable")
   3. VCPKG_DEFAULT_TRIPLET
      1. Wait for error message when run Build.bat in Refactored_ORB_SLAM2 folder.
      2. If you are using x64 system, it should be VCPKG_DEFAULT_TRIPLET
      3. !["add VCPKG_DEFAULT_TRIPLET path to environmental variable"](Doc\VCPKG_DEFAULT_TRIPLET.png " add VCPKG_DEFAULT_TRIPLET to environmental variable")
