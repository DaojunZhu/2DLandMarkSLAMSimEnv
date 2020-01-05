# 2DLandMarkSLAMSimEnv
A simple 2D LandMark based SLAM ROS simulation environment. 

It is aimed to test your own implementation of landmark based SLAM,for instance,2D EKF SLAM or graph based SLAM.

![img](https://github.com/DaojunZhu/2DLandMarkSLAMSimEnv/blob/master/slam_data_sim/sim.png)

**Red path:**  True path of robot motion

**Green path:**  Dead reckoning path from odometery information

**Gray circle around robot:** The measurement range of range bearing sensor

**Green dots: LandMarks Blue lines:** The measurements to current landmarks observed

### The meaningful output Topics

**"/NodeName/odom: "** The odometry measurement


**"/NodeName/observations:"** The current landmarks measurement, whos ROS message type is *slam_data_sim/RangeBearingObsData*


**slam_data_sim/RangeBearingObsData:** <br/>
  Header header <br/>
  int64[] ids <br/>
  float64[] ranges <br/>
  float64[] angles <br/>


**If you have any issues, please feel free to contack me through daojun@mail.ustc.edu.cn**
