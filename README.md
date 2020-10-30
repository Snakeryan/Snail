Our code is within the src (implementation) and include (header) files. For documentation of what the functions do is within the header files.

Even with the video, there are many more aspects to our program. These include using a Kalman filter (https://www.youtube.com/watch?v=CaCcOwJPytQ) on the IMU a
vision sensor readings, trying to first use a vision sensor to align to tower backboards, and taken in an anonymous function into our drive_to_point() function.

This function acts as a callback, meaning we can turn on and off specific motors at specific times when driving to a point.

We would like to give a shout out to Joey_24K as he made the algorithms that we use for our X-drive drivetrain (https://www.vexforum.com/t/x-drive-directional-drive-control-code-release-kind-of/82330).
