#
# Tasks for iRobot Create3 - Jazzy
# Created by scottcandy34
#

"""
Tasks for the iRobot Create3, which can be added to the TaskSchedular to run concurrently. 
Tasks are designed to run in the background and perform specific functions, such as generating 
coordinates for the robot to navigate to, or detecting columns in the environment. Tasks can 
be added to the TaskSchedular with a specified frequency, and can be stopped or started as needed.
"""

from .schedular import TaskSchedular
