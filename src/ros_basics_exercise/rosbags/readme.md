# How to play with recorded rosbag?

- rosbags can be downloaded from [Google Drive](https://drive.google.com/drive/folders/19KUzVqVasN7F2TfLpSc37OlQIdFQcbJs?usp=sharing) ([need EPFL account](http://gdrive.epfl.ch/))

-  the file structure is as follows:

    ```
    .
    ├── readme.md
    ├── real
    │   ├── thymio_real_with_obstacle_avoidance.bag
    │   └── thymio_real_without_obstacle.bag
    └── simu
        └── thymio_simulation_navigation_with_obstacle_avoidance.bag
    ```

- use `view_with_rosbag.launch` to visualize the annotated images from rosbag

    - default (visualize rosbag in simulation scene):

        ```shell
        roslaunch ros_basics_exercise set_simu_waypoints_obstacle.launch
        ```

    - visualize rosbag in real-world scene:

        ```shell
        roslaunch ros_basics_exercise set_simu_waypoints_obstacle.launch is_simu:=false
        ```

    - visualize with other bags in real world:

        ```shell
        roslaunch ros_basics_exercise view_with_rosbag.launch is_simu:=false real_bag_name:=thymio_real_without_obstacle
        ```
