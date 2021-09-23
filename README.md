# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
1. cmake >= 2.8
 * All OSes: [click here for installation instructions](https://cmake.org/install/)

2. make >= 4.1 (Linux, Mac), 3.81 (Windows)
 * Linux: make is installed by default on most Linux distros
 * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
 * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)

3. OpenCV >= 4.5
 * All OSes: refer to the [official instructions](https://docs.opencv.org/master/df/d65/tutorial_table_of_content_introduction.html)
 * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors. If using [homebrew](https://brew.sh/): `$> brew install --build-from-source opencv` will install required dependencies and compile opencv with the `opencv_contrib` module by default (no need to set `-DOPENCV_ENABLE_NONFREE=ON` manually). 

4. gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using either [MinGW-w64](http://mingw-w64.org/doku.php/start) or [Microsoft's VCPKG, a C++ package manager](https://docs.microsoft.com/en-us/cpp/build/install-vcpkg?view=msvc-160&tabs=windows). VCPKG maintains its own binary distributions of OpenCV and many other packages. To see what packages are available, type `vcpkg search` at the command prompt. For example, once you've _VCPKG_ installed, you can install _OpenCV 4.1_ with the command:
```bash
c:\vcpkg> vcpkg install opencv4[nonfree,contrib]:x64-windows
```
Then, add *C:\vcpkg\installed\x64-windows\bin* and *C:\vcpkg\installed\x64-windows\debug\bin* to your user's _PATH_ variable. Also, set the _CMake Toolchain File_ to *c:\vcpkg\scripts\buildsystems\vcpkg.cmake*.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

## Report
### MP.1 Data Buffer Optimization
To keep the data buffer size constant, one needs to remove the first element before adding a new data if the number of element is equal to the maximum desired data buffer size. This is called ring data buffer. Since the data buffer is a vector, one can use `erase` to delete an element. The implementation is written as follows:
```cpp
  // keep size of dataBuffer equals to dataBufferSize by erasing the first element
  if (dataBuffer.size() > dataBufferSize)
  {
      dataBuffer.erase(dataBuffer.begin());
  }

  dataBuffer.push_back(frame);
```

### MP.2 Keypoint Detection
The detector HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT are implemented based on the `detectorType` variable. The `detectorType` can be assigned from the `terminal input argument`, for instance user can execute `./2D_feature_tracking detectorType descriptorType`. If the `detectorType` is HARRIS, then HARRIS detector is selected, the implementation is written in `matching2D_Student.cpp`. If FAST, BRISK, ORB, AKAZE, or SIFT is selected, then function `detKeypointsModern` is called and the detector is selected accordingly. Note that for SIFT one needs to change the descriptor type as HOG instead of binary.

Each detector function return the execution time which will be used for MP.9.

### MP.3 Keypoint Removal
To focus on the determined vehicle, one needs to check if a point is contained within the region of intereset (ROI). To do so, a new vector `keypoints_roi` is created to store the points contained in the ROI. The ROI is determiend as `vehicleRect` variable, to check if the heypoint is in the ROI, one can use `vehicleRect.contains`. If the condition is true, than the point as appended to `keypoints_roi`.

### MP.4 Keypoint Descriptor
The `BRIEF, ORB, FREAK, AKAZE`, and `SIFT` descriptors are implemented based on the `descriptorType` variable. The `descriptorType` can be assigned from the `terminal input argument`, for instance user can execute `./2D_feature_tracking detectorType descriptorType`. The function `descKeypoints` at line 66 in`matching2D_Student.cpp` uses several `if` condition which is used to check which descriptor type the user wants to use. The execution time is returned by the function for MP.9 logging purpose.

### MP.5 Descriptor Matching
`FLANN` matching is implemented at line 20 in `matching2D_Student.cpp`. The `matcher` is created based on `cv::DescriptorMatcher::FLANNBASED`. User can select between `Brute Force` method or `FLANN Based` method by assigning the type to `matcherType` variable.
The selector type can be chosen between `NN` or `KNN` method by assigning the type into `selectorType` variable. For this particular task, 2 nearest neighbor is selected.

### MP.6 Descriptor Distance Ratio

Each match from the `knn_matches` vector is tested to check the distance. If the distance is less than `minDescDistRatio`, which is set to 0.8, the pair is appended to `matches` vector and the `matches` size is returned at the end of the function for MP.8 purpose.

### MP.7 Performance Evaluation 1 - Number of Keypoint

The number of keypoint on the region of interest (ROI) is assigned to `total_keypoints`. The number of keypoint is then recorded to a file `log/nb_of_keypoints.csv` if `LOGGING_KEYPOINTS` is set to `TRUE`. The file stores the keypoint detector type and number of keypoints deteted.

### MP.8 Performance Evaluation 2 - Number of Matches

The number of matches on all the image is assigned to `nb_of_matches`. The number of matches is then recorded to a file `log/nb_of_matches.csv` if `LOGGING_NB_OF_MATCHES` is set to `TRUE`. The file stores the keypoint detector and descriptor pair as well as the number of matches.



### MP.9 Performance Evaluation 3 - Execution Time
The execution time is calculated from the `time_detector` and `time_descriptor` which is returned by the keypoint detector and descriptor function. The number of matches is then recorded to a file `log/execution_time.csv` if `LOGGING_EXECUTION_TIME` is set to `TRUE`. The file stores the keypoint detector and descriptor pair as well as the execution time.

### Top Three Detector Descriptor Pair
One can choose number of matches divided by execution time as a performance evaluation. This describes the keypoints pair and how fast the process is calculated. Based on this criteria, the best pair is shown below:

|Sr. No. | Detector + Descriptor |Total Keypoints |Total Matches |Total Time (ms) |
|:---:|:---:|:----:|:-----:|:-----:|
|1 | FAST + BRIEF |1491 |11261 |139.7 |
|2 | FAST + ORB |1491 |10478 |171.1 |
|3 | FAST + BRISK |1491 |8742 |422.8 |

One should note that some detector cannot match `AKAZE` for the descriptor due to difference in method.


