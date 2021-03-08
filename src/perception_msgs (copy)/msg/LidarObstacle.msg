# polygen default -1
Header header
uint8 source			# 0: box (detection); 1: polygen (clustering); 2: others, such as roadedge.
geometry_msgs/Point position	# 3d position, detection only
float64 heading			# object heading, detection only
geometry_msgs/Point dimensions	# bounding box size, detection only
geometry_msgs/Point[] contours	# contour points in order, 4 points for detection
float64 minheight		# minimum range in z
float64 maxheight		# maximum range in z
int8 type			# classification result, detection only
int8 type_perception_car=0
int8 type_perception_pedestrian=1
int8 type_perception_cyclist=2
int8 type_perception_truck=3
int8 type_perception_forklift=4
int8 type_perception_golfcar=5
int8 type_perception_motorcyclist=6
int8 type_perception_bicycle=7
int8 type_perception_motorbike=8
float64 type_conf		# type score, detection only

