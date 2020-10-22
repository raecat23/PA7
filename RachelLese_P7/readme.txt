This code works through OpenCV and PID controllers. The program takes a photo from the source camera/rgb/image_raw 
and converts it to OpenCv so we can use the commands in OpenCV to read and analyze the files. From there, we mask the
image so that only yellow is showing, and everything else is black. The code uses the moments function from the OpenCV 
source to find the outline of the "blob" and the center of it. We then take a sliver of the image that is currently
showing, centered at the center of the circle, to further concentrate the line following algorithm. From there, we 
follow a PID steering algorithm to make sure the robot is correctly following the line. I took the PID algorithm from 
a previous PA that we have done to follow walls. 
