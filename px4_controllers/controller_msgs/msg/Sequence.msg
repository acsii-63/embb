# reference for polynomial trajectory tracking
#

std_msgs/Header header

uint8 ACTIVE = 0	# Position Velocity Acceleration Jerk Reference
uint8 FINNISH = 1	# Position Velocity Acceleration Reference
uint8 ACTIVE_FAILSAFE = 2	# Position Reference
uint8 FINNISH_FAILSAFE  = 3
uint8 status
uint8 seq
float32 progress

