# WSL2 Ubuntu20.04

sudo apt update

# INSTALL CMAKE if this is your first time using WSL 
sudo apt-get update
sudo apt-get install apt-transport-https ca-certificates gnupg software-properties-common wget

wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | sudo apt-key add -

sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main'
sudo apt-get update

sudo apt-get install cmake

# INSTALL Eigen3
sudo apt-get install libeigen3-dev

# INSTALL Boost
sudo apt-get install libboost-dev
## if got boost_filesystem version problem when build ORB-SLAM2, you also need to run
sudo apt install  libboost-filesystem1.71-dev

# INSTALL Opencv
sudo apt-get install libopencv-dev

# INSTALL Pangolin
# Go to any directory you want e.g. cd ~ to home dir
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..

# If got Pangolin build error: Could not find GLEW
# INSTALL GLEW
sudo apt-get install libglew-dev
# then recompile Pangolin

# Possible Issues

## CMAKE_CXX_COMPILER broken
## try
sudo apt-get update
sudo apt-get install -y build-essential
## if not work
install g++ or other C++ compiler


# If you have an error such that: Could NOT find OpenGL
sudo apt-get install libgl1-mesa-dev
# For an error, such that: Could NOT find OpenGL (missing: EGL)
sudo apt-get install libegl1-mesa-dev