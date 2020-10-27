#!/bin/bash
rosservice call /path_publisher/reset/vo "{}"
rosservice call /path_publisher/reset/wenc "{}"
rosservice call /path_publisher/reset/ekf "{}"
