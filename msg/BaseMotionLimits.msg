# Coordinates: x,y,z,roll,pitch,yaw. Translation bounds are offsets from the
# planning start position in world axes; angular bounds are absolute world RPY.
# Empty bounds add no pose restriction. Empty dynamic limits use server defaults.
float64[] lower_limits
float64[] upper_limits
float64[] max_velocity
float64[] max_acceleration
