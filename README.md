# ROS_noisifier
### Warning
 Bag1, Bag2, Bag3 don't work. They were hardcoded for ease of work and can be changed in scripts/gui/main2.py
In my case they looked like this:
```
PATHS = [
    "/home/mzhobro/filter_ws/src/localizationfusionlibrary/test/test1.bag",
    "/home/mzhobro/filter_ws/src/localizationfusionlibrary/test/test2.bag",
    "/home/mzhobro/filter_ws/src/localizationfusionlibrary/test/test3.bag"
]
```
GUI create several noised paths from one path(i.e. add noise to odom topics)

```
git clone https://gitlab.iavgroup.local/mzhobro/ros_noisifier.git
cd ros_noisifier
# run with
python scripts/gui/main2.py
```
Firstly choose the bag, nr of noised paths and topic you want to noisify:
![](./Images/1.PNG)


Than dynamically add noise to the paths
![](./Images/2.PNG)