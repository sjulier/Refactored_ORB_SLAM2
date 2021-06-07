# Refactored_ORB_SLAM2

This is a refactoring of the ORB_SLAM2 repository. It uses up-to-date cmake, up-to-date DBoW2 and g2o libraries, supports all static and all dynamic libraries (for debugging) and 2011 era C++ for sleep, threads, namespaces, etc.

You should be able to build by running:

./Build.sh



##### Difference on execution against original  ORB_SLAM2:

1.  The executable location is changed to  `./Build/${build_type}/Source/Examples/Monocular/`

   For instance, if you want to run mono_kitti with the default Build.sh file setting, the executable is located at `./Build/Debug/Source/Examples/Monocular/mono_kitti`

2. The setting file directory is prepended with default location at `./Source/Examples/`, therefore only filename is required for the yaml files.

3. Therefore, a example command for running KITTI00 dataset would be:

   `./Build/Debug/Source/Examples/Monocular/mono_kitti KITTI00-02.yaml ${your_kitti_dataset_folder}/sequences/00`
  
4. To simplify development, if you go to the directory:

   `cd ./Build/Debug/Source/Examples/Monocular`
   
   You can run the code from:
   
   `./mono_kitti KITTI00-02.yaml ${your_kitti_dataset_folder}/sequences/00`
  
   You can recompile code changes from:
   
   `make`
  
On linux it is highly recommend you get used to using valgrind. See https://www.valgrind.org/docs/manual/quick-start.html
   
   
