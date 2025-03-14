# ORB-SLAM2

This is a refactoring of the ORB_SLAM2 repository. It uses up-to-date cmake, up-to-date DBoW2, g2o and Pangolin libraries, supports all static and all dynamic libraries (for debugging) and C++17 multi-platform support for sleep, threads, namespaces, etc.

It has been succesfully built on Ubunutu (18.04-22.04), Windows 10 and 11 (WSL2), Intel Mac and M1 Mac. (The Windows 10 / 11 native build is currently broken; vcpkg and cmake do not interact properly and whether something builds or not changes on an hourly basis.)

## User-visible changes from the original ORB-SLAM2:

1.  All executables are installed in `./Install/bin` and the suffix "_d" is used to denote debug builds. For instance, if you want to run mono_kitti with the default (release) Build.sh file setting, the executable is at `./Install/bin/mono_kitti`. If you build with debugging enabled, the executable is `./Install/bin/mono_kitti_d`. Debug build is only required if you are doing some debugging and you are unlikely to need it for the coursework.

2. The ORB vocabulary is loaded automatically and does not have to be specified on the command line. (It is installed in the `./Install/var/lib/orbslam2` subdirectory). The system will try to load the binary version of the vocabulary. If it is not able to, it will load the text version, convert it, and save the binary version to the same directory. This speeds up start up times from several seconds to less than 0.5s.

3. The settings files are installed in `./Install/etc/orbslam2/`. By default, the executables will first search the current directory for the settings file and, if not defined, it will try the default directory. This means that there is less to type if running the standard examples (TUM, EuRoC, KITTI).
   
4. Some error checking is carried out on command line arguments to validate things like files and directories exist.
   
## Build instructions:

### Prerequisites

**Do not build the code in a conda environment; please deactivate it first otherwise you will end up trying to link with incompatible versions of libraries**

You will need to clone this repository using `git clone https://github.com/UCL/COMP0249_24-25_ORB_SLAM2.git`

It depends on a few widely-available libraries:

1. eigen3
2. boost
3. OpenCV (either 3.x or 4.x)
4. Suite sparse
5. GLEW
6. unzip
7. cmake (version 3.20 or above)

This repo ships with matched versions of DLib and DBoW2 (for the bag of words for data association), g2o (both front and backend optimization) and pangolin (GUI). The latter two are the most current release targets (as of 14/03/2025).

The build instructions are deliberately designed to be similar on all supported operating systems.

### Linux (and WSL2) build instructions

Install the dependencies:

`sudo apt install cmake build-essential libeigen3-dev libboost-dev libboost-filesystem-dev libblas-dev liblapack-dev libepoxy-dev libopencv-dev libglew-dev mesa-utils libgl1-mesa-glx unzip`

The default (Release) version of the library is built by running:

`./Build.sh`

To build a debug version, type:

`./Build.sh Debug`

If you want to avoid typing `./Install/bin` everywhere, run this command from the command line:

`export PATH=$PATH:$PWD/Install/bin`

#### Installing cmake 3.20:

If your version of cmake is older than 3.20, you will need to install the most recent version of cmake. There are several ways to do it. One is to use the current official release from kitware:

`wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | sudo apt-key add -`

`sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main'`

`sudo apt update`

`sudo apt install cmake`

The other is to follow instructions and build and install from source.

#### Display issues:

You can get errors of the form `terminate called after throwing an instance of 'std::runtime_error' what():  Pangolin X11: Failed to open X display`. To fix (at least in our case) set:

`export DISPLAY=:0`

### Mac (Intel and Apple Silicon) build instructions

We use `homebrew` (https://brew.sh/) and build using the XCode command line tools. Please ensure that both have been installed and that `brew doctor` is happy.

Install the dependencies:

`brew install cmake eigen boost suitesparse opencv glew`

You should be able to build the release by by running:

`./Build.sh`

To build a debug version, type:

`./Build.sh Debug`

If you want to avoid typing `./Install/bin` everywhere, run this command from the command line:

`export PATH=$PATH:$PWD/Install/bin`

### Windows 10/11 native build (does not work; do NOT use - use WSL instructions instead)

Windows 10/11 is a more challenging OS to build on because it doesn't have a completely standard location for development. We use `vcpkg` (https://github.com/microsoft/vcpkg) but other package management systems are available.

You need to install [git](https://git-scm.com/downloads), [Visual Studio](https://visualstudio.microsoft.com/vs/community/) and [vcpkg](https://learn.microsoft.com/en-us/vcpkg/get_started/get-started?pivots=shell-powershell).

After cloning the repository, install the vcpkg dependencies by running:

`vcpkg install`

This should create a directory `vcpkg_installed` in the current directory.

You should be able to build by running:

`Build.bat`

to build a release version. To build a debug version, type:

`Build.bat Debug`

Internally, cmake uses `Ninja`.

Set the path:

`set PATH=%PATH%;%cd%\vcpkg_installed\%VCPKG_TRIPLE%`

If you want to avoid typing `Install\bin everywhere`, modify the command to

`set PATH=%PATH%;%cd%\Install\bin;%cd%\vcpkg_installed\%VCPKG_TRIPLE%`

## Running Examples

ORB-SLAM2 builds multiple executables to handle data from the TUM, KITTI and EuRoC datasets for monocular (`mono_tum`, `mono_kitti`, `mono_euroc`), stereo (`stereo_kitti`, `stereo_euroc`) and RGBD (`rgbd_tum`) types of data. Each programme has a slightly different way of being invoked. The original instructions can be found on the [ORB-SLAM2 github page](https://github.com/raulmur/ORB_SLAM2). Modified versions are provided below.

Some of these methods of running the code seem to be fairly redundant, but we have included this redundancy to confirm with the original ORB-SLAM2 invocation.

We assume the instructions have been followed to set the `PATH` variable so you don't have to type the full Install path.

### Monocular SLAM

#### TUM Dataset

The data can be downloaded from [here](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download). Each dataset has a name of the form `fr_${fr_code}/${dataset_name}` on the webpage. When you download and uncompress it, you'll end up with a different folder name `${tum_dataset_folder}`. The command to run it is:

 `mono_tum TUM${fr_code}.yaml ${your_tum_dataset_folder} ${result_file_name}`

For example, suppose we wish to run `mono_tum` on the Freiburg `fr1/xyz dataset`. Download the data set from the repository directory (https://vision.in.tum.de/data/datasets/rgbd-dataset/download) and uncompress it to, say the same directory you cloned the repository into. This should produce a directory called  `rgbd_dataset_freiburg1_xyz`.

You can then run the code using:

`mono_tum TUM1.yaml rgbd_dataset_freiburg1_xyz fr01_results.txt` 

This will open the GUI, run the example, and write out a text file called `fr01_results.txt` which contains a time set of estiamtes of the camera pose.
   
#### KITTI Dataset

KITTI is a widely-used dataset for SLAM in outdoor environments. It was collected at Karlsruhe. More details can be found on the [KITTI website](https://www.cvlibs.net/datasets/kitti/index.php).

The dataset can be obtained from [Visual Odometry / SLAM Evaluation 2012](https://www.cvlibs.net/datasets/kitti/eval_odometry.php). It consists of a series of about 22 sequences (or runs) which were taken in different parts of the city under slightly different conditions. The downloads consist of all the data for a particular type of sensor for all the runs. 

**For mono, only the _grey-scale_ dataset should be downloaded.**

KITTI sequences are numbered 0,...,21. For run `${kitti_sequence}`, ORB-SLAM is invoked using the command line of the form:

 `mono_kitti KITTI${kitti_yaml_code}.yaml ${your_kitti_dataset_folder}/sequences/${kitti_sequence} ${result_file_name}`

In the 22 sequences, the first 11 (00-10) has ground truth for comparison. And ORB-SLAM embedded the lens calibration parameters for the first 13 sequences (00-12) in the yaml file it provided.
The `${kitti_yaml_code}` is determined as follows:

* For KITTI runs `00-02`: `00-02`
* For KITTI run `03`: `03`
* For KITTI runs `04-12`: `04-12`

For example, if you want to run KITTI sequence 05 and write the results in `kitti05_results.txt`, you would use the command:

 `mono_kitti KITTI04-12.yaml ${your_kitti_dataset_folder}/sequences/05 kitti05_results.txt`

#### EuRoC Dataset

The [EuRoC dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) is a challenging one which was collected for drone navigation. It consists of data collected from the drone (including camera and depth data) as well as very sophisticated ground-truth data from stand off outside-in tracking systems. There are two main sets of sequences: the machine hall (prefix MH) and the Vicon Room (prefix V).

All runs should be downloaded using the ASL format.

##### EuRoC Machine Hall Dataset

The Machine Hall datasets all have the form `MH${mh_sequence}`. The command is:

 `mono_euroc EuRoC.yaml ${your_euroc_dataset_folder}/MH${mh_sequence_number}/mav0/cam0/data MH${mh_sequence_number}.txt`

For example, for MH01 installed in the checkout directory, the command is:

`mono_euroc EuRoC.yaml MH01/mav0/cam0/data MH01.txt`

##### EuRoC Vicon Dataset

For a Vicon Room-related run  with sequence `${v_sequence}`, the command is:

 `mono_euroc EuRoC.yaml ${your_euroc_folder}/cam0/data ${mh_sequence}.txt`

### RGBD SLAM

#### TUM Dataset

This TUM mono dataset contains RGB as well. It can be downloaded from [here](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download). Each dataset has a name of the form `fr_${fr_code}/${dataset_name}` on the webpage. When you download and uncompress it, you'll end up with a different folder name `${tum_dataset_folder}`. The command to run it is:

`mono_tum TUM${fr_code}.yaml ${your_tum_dataset_folder} {associations_file_name}`

The associations file is used to link RGB and D files together. The files start with `fr_${fr_code}`, but there isn't a standard suffix that's used and has to be checked individuall. To run the basic set, an example would be:

`rgbd_tum TUM1.yaml rgbd_dataset_freiburg1_xyz fr1_xyz.txt`

### Stereo SLAM

Add down here (need directories for left and right frames).

#### EuRoC Dataset

All the EuRoC datasets actually include two sets of camera data (`cam0` and `cam1`) already. To run the code, you have to specify the directories containing both the left and right image streams. For example, for MH01 run

`stereo_euroc EuRoC.yaml MH_01/mav0/cam0/data MH_01/mav0/cam1/data MH01.txt`

#### KITTI Dataset

To do
