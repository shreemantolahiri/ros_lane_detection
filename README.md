# ros_lane_detection
gazebo simulation of lane detection using ebot

I have used world from aws-robomaker-challenge and made the ebot follow lane detection.
There are two approaches/methods:

* *Hough Lines Transforms*


* *FindContour*


You can find both of them in the **lane** folder.

To launch the ebot in the world run.
```roslaunch <package name> my_bot.launch```

*output videos:*

**findcontour output**


https://user-images.githubusercontent.com/92041385/157190039-4830df32-6818-41ef-a046-21f69d83271e.mp4

**hough lines transform ouput**



https://user-images.githubusercontent.com/92041385/157189980-1e1d8342-a55c-4ac3-a446-3a017cb16af8.mp4



