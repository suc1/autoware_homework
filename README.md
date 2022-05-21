# autoware_homework
Learn autoware.ai
## Ch2-RGBCloudProject
```
cd ~/autoware_homework/project1-RGBCloudmapping/RGBCloudProject
cmake .
make
```

## Ch3-Gnss map
```
#要先编译 "gnss" & "ndt_cpu", 最后编译 "gnss_projection"
# Move "gnss_projection" to another folder
cd project2-gnss_projection
catkin_make -DCATKIN_WHITELIST_PACKAGES="gnss_projection"

# Restore "gnss_projection" to ~
catkin_make -DCATKIN_WHITELIST_PACKAGES="gnss_projection"

source ./devel/setup.bash
roslaunch  gnss_projection  gnss_projection.launch

cd ~/autoware.ai/relative_files
rosbag play sample_moriyama_150324.bag
```
