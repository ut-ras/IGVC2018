Header header

float64 latitude
float64 longitude
float64 altitude

int32 num_satellites

# gives the velocities in north-east-down directions
# x is north velocity
# y is east  velocity
# z is down  velocity
geometry_msgs/Vector3 ned_velocities

# gives the acceleration in north-east-down directions
# x is north acceleration
# y is east  acceleration
# z is down  acceleration
geometry_msgs/Vector3 ned_acceleration

# the speed accuracy estimate in meters/sec
float64 speed_accuracy_estimate

# error checking information
bool error_present
string error_string
