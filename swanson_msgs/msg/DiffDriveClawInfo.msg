Header header

# Voltage of platform components, in volts
float64 roboclaw_voltage

# Component currents, in Amps
float64 left_motor_current
float64 right_motor_current

# Speed of motors, in quadrature pulses per second (qpps)
float64 left_motor_speed
float64 right_motor_speed

# Position of motors via encoders, in quadrature pulses (qp)
uint32 left_motor_position
uint32 right_motor_position

# 2D Pose based on Dead-reckoning via encoders
geometry_msgs/Pose2D estimated_pose

# Constants
float64 qpps_per_meter
float64 max_speed
float64 wheel_diameter
float64 base_width
