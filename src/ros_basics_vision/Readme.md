# ROS TP - Computer vision module

This package contains computer vision methods to:

- Adapt to lighting conditions dynamically and reduce image to a region of interest (the setup)
- Track orange ping-pong balls
- Track aruco markers on the robot

## Dependencies

- Opencv3 (with opencv-contrib)

## Adapting to lighting conditions

<img src="examples/images/adaptive_gamma.gif" alt="adaptive_gamma" width="640"/>

## Tracking orange ping-pong balls

<img src="examples/images/pingpong_tracking.gif" alt="ping_pong" width="640"/>

## Tracking aruco markers on the robot

<img src="examples/images/aruco_tracking.gif" alt="ping_pong" width="640"/>

## Compiling & running the examples

To compile the examples you should first invoke the following commands:

```shell
catkin_make -DBUILD_EXAMPLES=true
```

And then run the examples:

```shell 
# where X is the camera device (the first webcam would be 0) for your configuration

# Adaptive lighting
./adaptive_gamma_example X 

# Ball tracking
./ball_detector_example X 

# Aruco detection and pose estimation
./aruco_detector_example X 

# Setup detection
./ball_detector_example X 
```