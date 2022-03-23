# Refactored_ORB_SLAM2

This is a refactoring of the ORB_SLAM2 repository. It uses up-to-date cmake, up-to-date DBoW2 and g2o libraries, supports all static and all dynamic libraries (for debugging) and 2011 era C++ for sleep, threads, namespaces, etc.

You should be able to build by running:

`./Build.sh`

to build a debug version. To build a release version, type:

`./Build.sh Release`

If you want to avoid typing `./Install/bin` everywhere, run this command from the command line:

`set PATH=$PATH:$PWD/Install/bin`

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
   
`./mono_tum TUM1.yaml ~/Downloads/rgbd_dataset_freiburg1_xyz`
