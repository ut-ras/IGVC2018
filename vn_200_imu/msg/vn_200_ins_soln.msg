Header header

vn_200_imu/EulerOrientation orientation_euler

float64 geodetic_latitude
float64 geodetic_longitude
float64 altitude

# gives the velocities in north-east-down directions
# x is north velocity
# y is east  velocity
# z is down  velocity
geometry_msgs/Vector3 ned_velocities

# uncertainty in attitude estimate in degrees
float64 attitude_uncertainty

# uncertainty in position estimate in meters
float64 position_uncertainty

# uncertainty in velocity estimate in meters/sec
float64 velocity_uncertainty

# error checking information
bool error_present
string error_string
