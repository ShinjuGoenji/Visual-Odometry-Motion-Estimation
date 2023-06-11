# FPGA Implementation of Visual Odometry by Using High-Level Synthesis
[![](https://img.shields.io/badge/boledu-visual--odometry-brightgreen)](https://github.com/bol-edu/robotics-computing.git)
[![](https://img.shields.io/badge/vo--hls-paper-yellow)](https://implementation.ee.nthu.edu.tw/competition/groups/d654e3e1-c800-43e4-8583-01de78e7f9eb/attachments/summary?download=0)
[![](https://komarev.com/ghpvc/?username=ShinjuGoenji&color=red)](https://github.com/ShinjuGoenji/Visual-Odometry-Motion-Estimation.git)

This repository is for the project in the course **Special Project on Electrical Engineering**. 

It contains the code for our High-Level Synthesis (HLS) project on Visual Odometry. The Visual Odometry datasets are based on [*KITTI*](https://www.cvlibs.net/datasets/kitti/). The project was implemented using the [*Xilinx Vitis™ HLS*](https://www.xilinx.com/products/design-tools/vitis/vitis-hls.html) design flow and tested on the [*Alveo U50 Data Center Accelerator Cards*](https://www.xilinx.com/products/boards-and-kits/alveo/u50.html). 

Our goal is to accomplish an end-to-end service of accelerated visual odometry. It's our first time to learn High-Level Synthesis in Vitis design flow.

![](./doc/img/demo.gif)

*In this repos only contained the **motion estimation** part. Detailed documents of full system can be found [here](https://github.com/bol-edu/robotics-computing.git).*

## What's in Here
- [Toolchain and Prerequisites](#toolchain-and-prerequisites)
- [Prepare the Environment](#prepare-the-environment)
- [How to Build](#how-to-build)
- [How to Use](#how-to-use)
- [Algorithms](#algorithms)
- [Design Flow](#design-flow)

## Toolchain and Prerequisites
The project has been tested in the following environment:
- CPU: 11th Gen Intel(R) Core(TM) i7-11700 @ 2.50GHz
- RAM: 49112592kB DDR4
- OS: Ubuntu 20.04.4 LTS
- FPGA: Xilinx Alveo™ U50 FPGA
- Vitis Suite Version: 2022.1
- OpenCV Version: 4.4.0

Prerequisites:
- Ubuntu 20.04 (or a higher version certified with Vitis)
- Xilinx Vitis Suite 2022.1 (or a higher version)
- Xilinx® Runtime (XRT)
- CMake 3.5+
- OpenCV-4.4.0 x86 library (or a higher version certified with [Vitis Vision Library](https://github.com/Xilinx/Vitis_Libraries/tree/main/vision))
- libOpenCL.so
- libpng library (optional)

## Prepare the Environment
### 1. Xilinx Vitis Suite
Install necessary dependencies before Vitis installation: 
```
$ sudo apt install libtinfo5 libncurses5 -y
```
And follow the offical installation [guide](https://docs.xilinx.com/r/2022.1-English/ug1400-vitis-embedded/Installation-Requirements).

### 2. Xilinx® Runtime (XRT) 
Install XRT environment and set up variables.
>  [Installing Xilinx Runtime and Platforms](https://docs.xilinx.com/r/en-US/ug1393-vitis-application-acceleration/Installing-Xilinx-Runtime-and-Platforms)

1. **[Go to Alveo U50 Package File Downloads](https://www.xilinx.com/products/boards-and-kits/alveo/u50.html#gettingStarted)**</br>
Download and install them in order</br>
`  Xilinx Runtime  `
`  Deployment Target Platform  `
`   Development Target Platform  `


2. **Set up variables**</br>

    Set up XILINX_VITIS and XILINX_VIVADO variables
    ```
    $ source <Vitis_install_path>/Vitis/2022.2/settings64.sh 
    ``` 
    Set up XILINX_XRT for data center platforms
    ```  
    $ source /opt/xilinx/xrt/setup.sh  
    ```

### 3. CMake
Install the latest CMake (Make sure cmake version>3.5 before proceeding).
>  [Ubuntu cmake 編譯安裝](https://shengyu7697.github.io/ubuntu-cmake/)
>  [How to Install CMake on Ubuntu 20.04 LTS](https://vitux.com/how-to-install-cmake-on-ubuntu/

1. **[Go to CMake File Downloads](https://cmake.org/download/)** </br>
Download the source distribution `  cmake-<latest-version>.tar.gz  `
</br>

2. ****Remove previous CMake (If you have installed it before)***</br>
    Remove cmake and cmake-qt-gui
    ```
    $ sudo apt purge cmake
    ```
    Remove cmake and other dependencies
    ```
    $ sudo apt autoremove cmake
    ```

3. **Install it** </br>
    Extract it
    ```
    $ tar -zxvf cmake-<latest-version>.tar.gz
    ```
    Move to the extracted folder
    ```
    $ cd cmake-<latest-version>
    ```
    Run the following commands to compile and install
    ```
    $ ./bootstrap
    $ make
    $ sudo make install
    ```
    Check the installed CMace version
    ```
    $ cmake --version
    ```

### 4. OpenCV-4.4.0 x86 library
Libraries **should not** be builded at shared folder!
> [Install OpenCV-Python in Ubuntu](https://docs.opencv.org/4.4.0/d2/de6/tutorial_py_setup_in_ubuntu.html)


1. **Building OpenCV from source**</br>
    Required build dependencies 
    ```
    # CMake to configure the installation, GCC for compilation, Python-devel and Numpy for building Python bindings etc.
    $ sudo apt-get install cmake
    $ sudo apt-get install gcc g++

    # to support python3:
    $ sudo apt-get install python3-dev python3-numpy
    
    # GTK support for GUI features, Camera support (v4l), Media Support (ffmpeg, gstreamer) etc
    $ sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev
    $ sudo apt-get install libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev

    # to support gtk3:
    $ sudo apt-get install libgtk-3-dev
    ```
    We skip installation of 2 dependencies below: 
    `sudo apt-get install python-dev python-numpy` `sudo apt-get install libgtk2.0-dev` 
    
    And install optional dependencies
    ```
    $ sudo apt-get install libpng-dev
    $ sudo apt-get install libjpeg-dev
    $ sudo apt-get install libopenexr-dev
    $ sudo apt-get install libtiff-dev
    $ sudo apt-get install libwebp-dev
    ```

3. **[Download OpenCV-4.4.0 Repository](https://github.com/opencv/opencv/tree/4.4.0)**</br>
    Download whole repository into a folder named as `opencv`.</br>
    
    Open a terminal window and navigate to the downloaded `opencv` folder. 
    Create a new "build" folder and navigate to it.
    ```
    $ mkdir build
    $ cd build
    ```
4. **Configuring and Installing**</br>
    Configuration of OpenCV library build (executed from build folder)
    ```
    $ cmake ../
    ```
    OpenCV defaults assume `Release` build type and installation path is `/usr/local`.</br>
    
    You should see these lines in your CMake output (they mean that Python is properly found):
    ```
    --   Python 3:
    --     Interpreter:                 /usr/bin/python3.4 (ver 3.4.3)
    --     Libraries:                   /usr/lib/x86_64-linux-gnu/libpython3.4m.so (ver 3.4.3)
    --     numpy:                       /usr/lib/python3/dist-packages/numpy/core/include (ver 1.8.2)
    --     packages path:               lib/python3.4/dist-packages
    ```
    Now build the files
    ```
    $ make
    $ sudo make install
    ```
     All files are installed in `/usr/local/` folder. Open a terminal and try import `cv2`.
     ```
     import cv2 as cv
    print(cv.__version__)
    ```
    
    
### 5. libOpenCL.so</br>

> [OpenCL Installable Client Driver Loader](https://docs.xilinx.com/r/en-US/ug1393-vitis-application-acceleration/OpenCL-Installable-Client-Driver-Loader?tocId=rL1XqX3uRUq6DWvD71c6qQ)


* **Ubuntu**</br>
On Ubuntu the ICD library is packaged with the distribution. Install the following packages:
    ```
    $ sudo apt-get install ocl-icd-libopencl1
    $ sudo apt-get install opencl-headers
    $ sudo apt-get install ocl-icd-opencl-dev
    ```
    </br>
    
### 6. libpng library (optional)

> [Official Repository Installation Guide](https://github.com/glennrp/libpng/blob/libpng16/INSTALL)
> [How to install libpng-1.6.37.tar.xz in ubuntu 20.04?](https://askubuntu.com/questions/1267837/how-to-install-libpng-1-6-37-tar-xz-in-ubuntu-20-04)


1. **[Download libpng Repository](https://github.com/glennrp/libpng)**</br>
    Download whole repository into a temporary folder,
    or type GitHub CLI command under a temporary folder
    ```
    $ gh repo clone glennrp/libpng
    ```
    
2. **Configuring and Installing**</br>
    ```
    $ cd libpng
    $ ./autogen.sh
    $ ./configure --prefix=/usr/local/libpng
    $ make check
    $ sudo make install
    ```
   
Appendix

`< path-to-opencv-lib-folder >` = `/usr/local/lib`

`< path-to-opencv-include-folder >` = `/usr/local/include/opencv4`

`< path-to-platform-directory >/< platform >.xpfm` = `/opt/xilinx/platforms/xilinx_u50_gen3x16_xdma_5_202210_1/xilinx_u50_gen3x16_xdma_5_202210_1.xpfm`

## How to Build
1. clone the repos
```
git clone https://github.com/ShinjuGoenji/Visual-Odometry-Motion-Estimation.git
```

1. ```cd``` to directory [program](./program/).
```
cd ./program
```
2. ```make``` the host executable program & xclbin files using Makefile.
```
make all TARGET=hw
```
3. As long as the host program & kernel are compiled, you may test the functionality of the kernel, using [test data](./program/testdata/). \
Run the program using
```
make run TARGET=hw
```
4. (option) You may modify the arguements by [args.mk](./program/args.mk)

## How to Use
### Kernel Interface

|  #  | Arguments  |  Type  | Size (number of items) | input/ouput  |
| :-: | :--------: | :----: | :--------------------: | :----------: |
|  0  |  matches   | MATCH  |    MAX_KEYPOINT_NUM    |    input     |
|  1  | match_num  |  int   |           1            |    input     |
|  2  |    kp0     | IPOINT |    MAX_KEYPOINT_NUM    |    input     |
|  3  |    kp1     | IPOINT |    MAX_KEYPOINT_NUM    |    input     |
|  4  |     fx     | float  |           1            |    input     |
|  5  |     fy     | float  |           1            |    input     |
|  6  |     cx     | float  |           1            |    input     |
|  7  |     cy     | float  |           1            |    input     |
|  8  |   depth    | float  | IMG_WIDTH * IMG_HEIGHT |    input     |
|  9  | threshold  |  int   |           1            |    input     |
| 10  | confidence | float  |           1            |    input     |
| 11  |  maxiter   |  int   |           1            |    input     |
| 12  |    rmat    | float  |           9            | input/output |
| 13  |    tvec    | float  |           3            | input/output |
| 14  | take_last  |  bool  |           1            |    input     |

### Arguements Description
* matches   \
    An array of matched indices of kp0 and kp1 from ***Feature Matching***. The maximum buffer size is MAX_KEYPOINT_NUM.

* match_num \
    Number of matched keypoint sets.

* kp0, kp1 \
    Keypoint 0, Keypoint 1 from ***Feature Extraction***.

* fx, fy, cx, cy \
    Focal length, optical center from ***intrinsic matrix***. \
    ```fx = K_left[0][0], fy = K_left[1][1], cx = K_left[0][2], cy = K_left[1][2]```.

* depth \
    Depth map from ***Stereo Matching***.

* threshold \
    Parameter for RANSAC. Distance (in pixel) to determine whether the projected 2D point is outlier.

* confidence \
    Parameter for RANSAC. To determine whether the number of inlier is sufficient.

* maxiter \
    Parameter for RANSAC. The maximum number of iteration to operate RANSAC.

* rmat, tvec \
    Outcome rotation matrix and translation vector.

* take_last \
    To determine whether rmat and tvec are taken as inputs to act as initial values of gradient descent.


### Type / Marco Description
* MAX_KEYPOINT_NUM
    ``` cpp
    #define MAX_KEYPOINT_NUM 500
    ```
    
* MATCH
    ``` cpp
    struct MATCH
    {
        int a;      // index of kp0
        int b;      // index of kp1
    };
    ```
* IPOINT
    ``` cpp
    struct IPOINT
    {
        float x;    // x coordinate of 2D point
        float y;    // y coordinate of 2D point
    };
    ```

## Design Flow

## Algorithms

![](./doc/img/algorithm%20flow.gif)

