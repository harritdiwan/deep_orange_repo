# Custom message. This message will contain description, pose, orientation, velocity and bounding polygons for the detected obstacles. 

# String describing the detected obstacle
string description

# Global position and velocity of the obstacle	
nav_msgs/Odometry pose_velocity

# Bounding points of the obstacle
geometry_msgs/Polygon shape		

# Estimated trajectory of the obstacle
nav_msgs/Path trajectory		

