# `zva_sd0_kisbeadando` package
ROS 2 C++ package.  [![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)](https://docs.ros.org/en/humble/)
## Packages and build

It is assumed that the workspace is `~/ros2_ws/`.

### Clone the packages
``` r
cd ~/ros2_ws/src
```
``` r
git clone https://github.com/cvada/zva_sd0_kisbeadando
```

### Build ROS 2 packages
``` r
cd ~/ros2_ws
```
``` r
colcon build --packages-select zva_sd0_kisbeadando --symlink-install
```

<details>
<summary> Don't forget to source before ROS commands.</summary>

``` bash
source ~/ros2_ws/install/setup.bash
```
</details>

```
ros2 launch zva_sd0_kisbeadando launch_example1.launch.py
```

ROS 2 pacage template, to get started, use template by clicking on the Green button labeled [`Use this template`](https://github.com/cvada/zva_sd0_kisbeadando/generate) / [`Create new repository`](https://github.com/cvada/zva_sd0_kisbeadando/generate). 

<p align="center"><img src="img/use_this_template01.png" width="60%" /></p>


Let's assume 
- your Github username is `cvada`
- your ROS 2 repo shold be `zva_sd0_kisbeadando`

Replace everything in the cloned repo:

- `zva_sd0_kisbeadando` >> `cool_ros2_package` (the folder was already renamed after `Use this template`)
- `cvada` >> `mycoolname`
- find all `todo` strings and fill the blanks

The easiest way is VS code:

<p align="center"><img src="img/replace01.png" width="60%" /></p>

Now `colcon build` your ROS 2 package and you can start wokring.