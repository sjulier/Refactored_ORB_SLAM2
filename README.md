# Refactored_ORB_SLAM2

This is a refactoring of the ORB_SLAM2 repository. It uses up-to-date cmake, up-to-date DBoW2 and g2o libraries, supports all static and all dynamic libraries (for debugging) and 2011 era C++ for sleep, threads, namespaces, etc.

It has been succesfully built on Ubunutu (18.04 and 20.04), Windows 10 (natively and WSL2), Windows 11 (natively and WSL2), Intel Mac and M1 Mac.


Please refer to repo wiki for detailed guidance of dependency installation and compilation of individual platforms:

https://github.com/sjulier/Refactored_ORB_SLAM2/wiki


## Difference on execution against original  ORB_SLAM2:

1.  All executables are installed in `./Install/bin` and the suffix "_d" is used to denote debug builds.

   For instance, if you want to run mono_kitti with the default Build.sh file setting, the executable is at `./Install/bin/mono_kitti`. If you build with debugging enabled, the executable is at `./Install/bin/mono_kitti_d`.

2. The standard ORB vocabulary is loaded by default and does not have to be specified on the command line. (It is installed in `./Install/var/lib/orbslam2` subdirectory). The system will try to load the binary version of the vocabulary. If it is not able to, it will load the text version, convert it, and save the binary version to the same directory. This speeds up start up times from several seconds to less than 0.5s.

3. The standard settings files are installed in `./Install/etc/orbslam2/Monocular`. You do not need to specify the directory. Therefore, a example command for running KITTI00 (using a debug build) is:
   
   (replace `00` with your sequence number)
   
   (a text files such as `result.txt` for result_file_name is perferred as you can open the file later in the text editor directly.):

 `./Install/bin/mono_kitti_d KITTI00-02.yaml ${your_kitti_dataset_folder}/sequences/00 ${result_file_name}`
   
 The release build would be:
 
   `./Install/bin/mono_kitti KITTI00-02.yaml ${your_kitti_dataset_folder}/sequences/00 ${result_file_name}`
   
If you have set the `PATH` variable as specified above, you can change it to:

   `mono_kitti KITTI00-02.yaml ${your_kitti_dataset_folder}/sequences/00 ${result_file_name}`

On Linux it is highly recommend you should become familiar with `valgrind`. See https://www.valgrind.org/docs/manual/quick-start.html

##### Other examples

Suppose we wish to run `mono_tum` on the Freiburg fr1/xyz dataset. Download the data set from the repository directory (https://vision.in.tum.de/data/datasets/rgbd-dataset/download) and uncompress it to, say your `~/Downloads` directory. This should produce a directory called  `rgbd_dataset_freiburg1_xyz`.

You can then run the code using:

`./Install/bin/mono_tum TUM1.yaml ~/Downloads/rgbd_dataset_freiburg1_xyz ${result_file_name}`   
   
 Or, if you did the trick with setting `PATH`, this can be simplified to:
   
`./mono_tum TUM1.yaml ~/Downloads/rgbd_dataset_freiburg1_xyz ${result_file_name}`




Build instructions:
-----

## Prerequisites

This version depends on a few system libraries:

1. eigen3
2. boost
3. OpenCV (either 3.x or 4.x)
4. Suite sparse

The code comes shipped with matched versions of DLib and DBoW2 (for the bag of words for data association), g2o (both front and backend optimization) and pangolin (GUI).

The build instructions are deliberately designed to be similar on all supported operating systems.

### Linux build instructions


Clone this repository.

Install dependencies:

`sudo apt install boost eigen suitesparse opencv`

You should be able to build by running:

`./Build.sh`

to build the release version. To build a debug version, type:

`./Build.sh Debug`

If you want to avoid typing `./Install/bin` everywhere, run this command from the command line:

`set PATH=$PATH:$PWD/Install/bin`


### Mac (Intel and Apple Silicon) build instructions.

We use homebrew and build using the XCode command line tools. Please ensure that both have been installed.

Install the support libraries:

`brew install boost eigen suitesparse opencv`

Clone this repository.

You should be able to build the release by by running:

`./Build.sh`

To build a debug version, type:

`./Build.sh Debug`

If you want to avoid typing `./Install/bin` everywhere, run this command from the command line:

`set PATH=$PATH:$PWD/Install/bin`

### Windows 10/11 build

Windows 10/11 is a more challenging OS to build on because it doesn't have hard standards for how to lay out build and install. We use `vcpkg` (https://github.com/microsoft/vcpkg).

PLEASE MAKE SURE TO CLEAR THE CMAKE PACKAGE REGISTRY BEFORE YOU TRY TO BUILD. WE HAVE ENCOUNTERED A LOT OF PROBLEMS WITH THE PACKAGE REGISTRY BEING USED TO LINK TO THE WRONG VERSIONS OF LIBRARIES, LEADING TO A LOT OF CONFUSION AND FRUSTRATION.

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



