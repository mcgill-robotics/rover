###### TODO: Need to setup odrive for adequate arm movement. ######

#--------------------------------------------------------------------------------------------#
#
# Goal: Write a script that contains the arm odrive node that adequately
# controls the arm's odrive requirements. 
#
# Tasks: 
#       1. Understand the arm's odrive requirements and translate them to odrive setup.
#           a. Controller state (velocity, position, ramp velocity, ...)
#           b. velocity, position, acceleration, current and other parameter limits if needed.
#       2. Create a node class that contains all the necessary information to write a loop
#          to keep the arm functional and moving.
#       3. Test odrive singularly to validate parameters.
#       4. Test odrive with arm to validate parameters and ensure proper functioning.
#
#--------------------------------------------------------------------------------------------#