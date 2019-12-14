# The control law is as follows:
#
# command = efforts + position_gains * (positions - q_measured) + velocity_gains * (velocities - qdot_measured)
#
float64[] positions
float64[] velocities
float64[] efforts
float64[] position_gains
float64[] velocity_gains
