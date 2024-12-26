#region VEXcode Generated Robot Configuration
from vex import *
import urandom

##
# Brain should be defined by default
brain=Brain()

# Robot configuration code
controller_1 = Controller(PRIMARY)
Intake = Motor(Ports.PORT7, GearSetting.RATIO_18_1, False)
inertial_8 = Inertial(Ports.PORT8)


# wait for rotation sensor to fully initialize
wait(30, MSEC)


def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")



# define variables used for controlling motors based on controller inputs
controller_1_right_shoulder_control_motors_stopped = True

# define a task that will handle monitoring inputs from controller_1
def rc_auto_loop_function_controller_1():
    global controller_1_right_shoulder_control_motors_stopped, remote_control_code_enabled
    # process the controller input every 20 milliseconds
    # update the motors based on the input values
    while True:
        if remote_control_code_enabled:
            # check the buttonR1/buttonR2 status
            # to control Intake
            if controller_1.buttonR1.pressing():
                Intake.spin(FORWARD)
                controller_1_right_shoulder_control_motors_stopped = False
            elif controller_1.buttonR2.pressing():
                Intake.spin(REVERSE)
                controller_1_right_shoulder_control_motors_stopped = False
            elif not controller_1_right_shoulder_control_motors_stopped:
                Intake.stop()
                # set the toggle so that we don't constantly tell the motor to stop when
                # the buttons are released
                controller_1_right_shoulder_control_motors_stopped = True
        # wait before repeating the process
        wait(20, MSEC)

# define variable for remote controller enable/disable
remote_control_code_enabled = True

rc_auto_loop_thread_controller_1 = Thread(rc_auto_loop_function_controller_1)

#endregion VEXcode Generated Robot Configuration

# ------------------------------------------
# 
# 	Project:      ALOOminium
#	Author:       Neel Paul
#	Created:      3/29/2024
#	Description:  VEXcode V5 Python Project
# 
# ------------------------------------------

# Library imports
from vex import *

# Begin project code
# 6-motor drivetrain initialization
left_motor_a = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)
left_motor_b = Motor(Ports.PORT2, GearSetting.RATIO_6_1, False)
left_motor_c = Motor(Ports.PORT3, GearSetting.RATIO_6_1, False)
left_drive_smart = MotorGroup(left_motor_a, left_motor_b, left_motor_c)
right_motor_a = Motor(Ports.PORT4, GearSetting.RATIO_6_1, False)
right_motor_b = Motor(Ports.PORT5, GearSetting.RATIO_6_1, False)
right_motor_c = Motor(Ports.PORT6, GearSetting.RATIO_6_1, False)
right_drive_smart = MotorGroup(right_motor_a, right_motor_b, right_motor_c)
drivetrain = DriveTrain(left_drive_smart, right_drive_smart, 319.19, 295, 40, MM, 1)
drivetrain_l_needs_to_be_stopped_controller_1 = False
drivetrain_r_needs_to_be_stopped_controller_1 = False
while True:     

    if remote_control_code_enabled:
        
        #left motors forward - negative axis 3 position
        #right motors forward - positive axis 3 position
        drivetrain_left_side_speed = -controller_1.axis3.position() - controller_1.axis1.position()
        drivetrain_right_side_speed = controller_1.axis3.position() - controller_1.axis1.position()
            
        # check if the value is inside of the deadband range
        if drivetrain_left_side_speed < 5 and drivetrain_left_side_speed > -5:
            # check if the left motor has already been stopped
            if drivetrain_l_needs_to_be_stopped_controller_1:
                # stop the left drive motor
                left_drive_smart.stop()
                # tell the code that the left motor has been stopped
                drivetrain_l_needs_to_be_stopped_controller_1 = False
        else:
            # reset the toggle so that the deadband code knows to stop the left motor next
            # time the input is in the deadband range
            drivetrain_l_needs_to_be_stopped_controller_1 = True
        # check if the value is inside of the deadband range
        if drivetrain_right_side_speed < 5 and drivetrain_right_side_speed > -5:
            # check if the right motor has already been stopped
            if drivetrain_r_needs_to_be_stopped_controller_1:
                # stop the right drive motor
                right_drive_smart.stop()
                # tell the code that the right motor has been stopped
                drivetrain_r_needs_to_be_stopped_controller_1 = False
        else:
            # reset the toggle so that the deadband code knows to stop the right motor next
            # time the input is in the deadband range
            drivetrain_r_needs_to_be_stopped_controller_1 = True
            
        # only tell the left drive motor to spin if the values are not in the deadband range
        if drivetrain_l_needs_to_be_stopped_controller_1:
            left_drive_smart.set_velocity(drivetrain_left_side_speed, PERCENT)
            left_drive_smart.spin(FORWARD)
        # only tell the right drive motor to spin if the values are not in the deadband range
        if drivetrain_r_needs_to_be_stopped_controller_1:
            right_drive_smart.set_velocity(drivetrain_right_side_speed, PERCENT)
            right_drive_smart.spin(FORWARD)
