# FPGA Implementation of Visual Odometry by Using High-Level Synthesis
[![](https://img.shields.io/badge/boledu-visual--odometry-brightgreen)](https://github.com/bol-edu/robotics-computing.git)
[![](https://img.shields.io/badge/vo--hls-paper-brightgreen)](https://implementation.ee.nthu.edu.tw/competition/groups/d654e3e1-c800-43e4-8583-01de78e7f9eb/attachments/summary?download=0)

This repository is for the project in the course 
**Special Project on Electrical Engineering**. 

It contains the code for our High-Level Synthesis (HLS) project on Visual Odometry. The Visual Odometry datasets are based on [*KITTI*](https://www.cvlibs.net/datasets/kitti/). The project was implemented using the [*Xilinx Vitis™ HLS*](https://www.xilinx.com/products/design-tools/vitis/vitis-hls.html) design flow and tested on the [*Alveo U50 Data Center Accelerator Cards*](https://www.xilinx.com/products/boards-and-kits/alveo/u50.html). 

Our goal is to accomplish an end-to-end service of accelerated visual odometry. It's our first time to learn High-Level Synthesis in Vitis design flow.

*In this repos only contained the **motion estimation** part. Detailed documents of full system can be found [here](https://github.com/bol-edu/robotics-computing.git).*

![](./doc/img/algorithm%20flow.gif)

## What's in Here
- [How to Build](#how-to-build)
- [How to Use](#how-to-use)
- [Algorithms](#algorithms)
- [Design Flow](#design-flow)

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


