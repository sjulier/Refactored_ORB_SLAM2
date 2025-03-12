# Refactored_ORB_SLAM2

This is a refactoring of the ORB_SLAM2 repository. It uses up-to-date cmake, up-to-date DBoW2, g2o and Pangolin libraries, supports all static and all dynamic libraries (for debugging) and 2011 era C++ for sleep, threads, namespaces, etc.

It has been succesfully built on Ubunutu (18.04-22.04), Windows 10 and 11 (natively and on WSL2), Intel Mac and M1 Mac.

## Difference of execution against the original ORB_SLAM2:

1.  All executables are installed in `./Install/bin` and the suffix "_d" is used to denote debug builds.

   For instance, if you want to run mono_kitti with the default (release) Build.sh file setting, the executable is at `./Install/bin/mono_kitti`. If you build with debugging enabled, the executable is at `./Install/bin/mono_kitti_d`. Typically you will only need to build Debug if there are issues with execution.

2. The ORB vocabulary is loaded automatically and does not have to be specified on the command line. (It is installed in the `./Install/var/lib/orbslam2` subdirectory). The system will try to load the binary version of the vocabulary. If it is not able to, it will load the text version, convert it, and save the binary version to the same directory. This speeds up start up times from several seconds to less than 0.5s.

3. The standard settings files are installed in `./Install/etc/orbslam2/`. You do not need to specify the directory. Therefore, a example command for running KITTI00 (using a debug build) is:
   
   (replace `00` with your sequence number)
   
   (a text files such as `result.txt` for result_file_name is perferred as you can open the file later in the text editor directly.):

 `./Install/bin/mono_kitti_d KITTI00-02.yaml ${your_kitti_dataset_folder}/sequences/00 ${result_file_name}`
   
 The release build would be:
 
   `./Install/bin/mono_kitti KITTI00-02.yaml ${your_kitti_dataset_folder}/sequences/00 ${result_file_name}`
   
If you have set the `PATH` variable as specified above, you can change it to:

   `mono_kitti KITTI00-02.yaml ${your_kitti_dataset_folder}/sequences/00 ${result_file_name}`

##### Running examples

Suppose we wish to run `mono_tum` on the Freiburg fr1/xyz dataset. Download the data set from the repository directory (https://vision.in.tum.de/data/datasets/rgbd-dataset/download) and uncompress it to, say your `~/Downloads` directory. This should produce a directory called  `rgbd_dataset_freiburg1_xyz`.

You can then run the code using:

`./Install/bin/mono_tum TUM1.yaml ~/Downloads/rgbd_dataset_freiburg1_xyz ${result_file_name}`   
   
 Or, if you did the trick with setting `PATH`, this can be simplified to:
   
`mono_tum TUM1.yaml ~/Downloads/rgbd_dataset_freiburg1_xyz ${result_file_name}`


Build instructions:
-----

## Prerequisites

This version depends on a few system libraries:

1. eigen3
2. boost
3. OpenCV (either 3.x or 4.x)
4. Suite sparse
5. GLEW
6. unzip

The code comes shipped with matched versions of DLib and DBoW2 (for the bag of words for data association), g2o (both front and backend optimization) and pangolin (GUI).

The build instructions are deliberately designed to be similar on all supported operating systems.

### Linux build instructions


Clone this repository.

Install dependencies:

`sudo apt install cmake build-essential libeigen3-dev libboost-dev libboost-filesystem-dev libblas-dev liblapack-dev libopencv-dev libglew-dev mesa-utils libgl1-mesa-glx unzip`

You should be able to build by running:

`./Build.sh`

to build the release version. To build a debug version, type:

`./Build.sh Debug`

If you want to avoid typing `./Install/bin` everywhere, run this command from the command line:

`set PATH=$PATH:$PWD/Install/bin`


### Mac (Intel and Apple Silicon) build instructions.

We use `homebrew` (https://brew.sh/) and build using the XCode command line tools. Please ensure that both have been installed.

Install the support libraries:

`brew install eigen boost suitesparse opencv glew`

Clone this repository.

You should be able to build the release by by running:

`./Build.sh`

To build a debug version, type:

`./Build.sh Debug`

If you want to avoid typing `./Install/bin` everywhere, run this command from the command line:

`set PATH=$PATH:$PWD/Install/bin`

### Windows 10/11 build

Windows 10/11 is a more challenging OS to build on because it doesn't have hard standards for how to lay out build and install. We use `vcpkg` (https://github.com/microsoft/vcpkg) but other package management systems are available.

First install a copy of Visual Studio (e.g., Community 2022).

Second, install `vcpkg`. See (https://learn.microsoft.com/en-us/vcpkg/get_started/get-started?pivots=shell-powershell)

Third, install the vcpkg dependencies by running:

`vcpkg install`

This should create a directory `vcpkg_installed` in the current directory.

You should be able to build by running:

`Build.bat`

to build a release version. To build a debug version, type:

`Build.bat Debug`

This uses the `Ninja` build system.

Set the path:

`set PATH=%PATH%;%cd%\vcpkg_installed\%VCPKG_TRIPLE%`



