- ### What we have
  ***depth map*** from Stereo Matching

  matched 2D ***keypoints*** of 1st and 2nd left images

  ***camera intrinsic matrix*** 

  Now we are looking for Rotation matrix ***rmat*** and translation vector ***tvec*** to estimate the relative motion between 1st and 2nd images. 

  ```cpp
  void estimate(Matrix &match, Matrix &kp0, Matrix &kp1, Matrix &k, Matrix &depth, Matrix &rmat, Matrix &tvec);
  ```

- ### Projection
  Before doing the math part, we need to modify the input to match the requirement. Because we're using PnP, which solves relative motion cv problem by 2D-3D points. As a result, we project 1st left image's keypoints to 3D point through depth map and camera matrix. Also we align 2 sets of keypoints through matched index.

<p align="center">
  <img src="../img/Motion%20Estimation%20block%20diagram.png" />
  <img src="../img/EPnP%20block%20diagram.png" />
</p>