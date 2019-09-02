Header header

# Voltage of platform components, in volts
float64 left_roboclaw_voltage
float64 right_roboclaw_voltage

# Component currents, in Amps
float64[2] left_motor_currents
float64[2] right_motor_currents

# Speed of motors, in quadrature pulses per second (qpps)
float64[2] left_motor_speeds
float64[2] right_motor_speeds

# Position of motors via encoders, in quadrature pulses (qp)
uint32[2] left_motor_positions
uint32[2] right_motor_positions

# 2D Pose based on Dead-reckoning via encoders
geometry_msgs/Pose2D estimated_pose

# Constants
float64 qpps_per_meter
float64 max_speed
float64 wheel_diameter
float64 base_width
