# IMU-Analysis
UROP Code for ACL, analyzing the performance of a drone IMU when compared to the measurements obtained by the lab's motion capture system.
Makes use of matlab_utilities and matlab-utils repositories.  Run code on bag files of recorded data.
Still a work in progress, working on improving the display of the outputs
An overview of the script outputs:
1.) Plots drone position and velocity in three dimensions
2.) Plots IMU vs. Motion Capture data for phi and theta attitudes along with gyroscope measurements
3.) Calculates RMSE for all measurements and displays on corresponding graph
4.) Analyzes the noise in IMU data broken up into thrust-on vs thrust-off time periods.  Displays a vector with the STD of the calculated 
zero mean error.  (-10 is returned if the time period didn't have enough data points to approximate as gaussian)
5.) Returns a vector of the bias terms used when calculating the zero-mean error for each attitude measurement.
