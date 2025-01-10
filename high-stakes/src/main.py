# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Neel Paul                                                    #
# 	Created:      10/27/2024, 3:50:54 PM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default + defining initial variables
brain = Brain()
inertial = Inertial(Ports.PORT20) #Example port, change after building robot
controller_1 = Controller(PRIMARY)
intake_chain = Motor(Ports.PORT7, GearSetting.RATIO_6_1, False)
intake_arm = Motor(Ports.PORT8, GearSetting.RATIO_18_1, False)
intake = MotorGroup(intake_chain, intake_arm)
intake.set_velocity(100, PERCENT) #Make sure intake always runs at full speed
enableturnPID = False #Condition for turn PID loop
enablePID = False
goal_clamp = DigitalOut(brain.three_wire_port.h)

#6-motor drivetrain initialization
left_motor_a = Motor(Ports.PORT1, GearSetting.RATIO_6_1, True)
left_motor_b = Motor(Ports.PORT2, GearSetting.RATIO_6_1, True)
left_motor_c = Motor(Ports.PORT3, GearSetting.RATIO_6_1, True)
left_drive_smart = MotorGroup(left_motor_a, left_motor_b, left_motor_c)
right_motor_a = Motor(Ports.PORT4, GearSetting.RATIO_6_1, False)
right_motor_b = Motor(Ports.PORT5, GearSetting.RATIO_6_1, False)
right_motor_c = Motor(Ports.PORT6, GearSetting.RATIO_6_1, False)
right_drive_smart = MotorGroup(right_motor_a, right_motor_b, right_motor_c)
drivetrain = DriveTrain(left_drive_smart, right_drive_smart, 299.24, 273.05, 266.7, MM, 0.75)
#drivetrain1 = DriveTrain(left_drive_smart, right_drive_smart, 260, idk, idk, MM, 0.75)


def input_monitoring():
   drivetrain_l_needs_to_be_stopped_controller_1 = False
   drivetrain_r_needs_to_be_stopped_controller_1 = False
   intake_stopped = True
   goal_clamp_clamped = False
   while True:


       if remote_control_code_enabled:
      
           #left motors forward - negative axis 3 position
           #right motors forward - positive axis 3 position
           drivetrain_left_side_speed = controller_1.axis3.position() + controller_1.axis1.position()
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
            
           #conditions to continously check for right shoulder button input and spin intake if so
           if controller_1.buttonR1.pressing():
               intake.spin(FORWARD)
               intake_stopped = False
           elif controller_1.buttonR2.pressing():
               intake.spin(REVERSE)
               intake_stopped = False
           elif not intake_stopped:
               intake.stop()

           if controller_1.buttonL1.pressing():
                if not goal_clamp_clamped:
                    goal_clamp.set(True)
                    goal_clamp_clamped = True
                    wait(250, MSEC)
                elif goal_clamp_clamped:
                    goal_clamp.set(False)
                    goal_clamp_clamped = False
                    wait(250, MSEC)
                
remote_control_code_enabled = True
control_loop = Thread(input_monitoring)

def pre_auton():
    inertial.calibrate()
    wait(2, SECONDS)
    inertial.set_rotation(0, DEGREES)

def auton():
    pre_auton()

def user_control():
    user_control_loop = Thread(input_monitoring)

#Function for lateral PID, needs thread
def PIDControlFn(target):
    kP = 0 #tune
    kI = 0 #tune
    kD = 0 #tune

    derivative = 0
    preverror = 0
    totalerror = 0

    target = target / 260

    left_drive_smart.set_position(0, DEGREES)
    right_drive_smart.set_position(0, DEGREES)

    while enablePID:
        average_position = (left_drive_smart.position(DEGREES) + right_drive_smart.position(DEGREES)) / 2

        error = target - average_position

        derivative = error - preverror

        totalerror += error

        motor_power = (error * kP + derivative * kD + totalerror * kI)

        left_drive_smart.spin(FORWARD, motor_power, RPM) #fix
        right_drive_smart.spin(FORWARD, motor_power, RPM) #fix

        preverror = error
        wait(50, MSEC)

#Function for turn PID, needs thread
def TurnPIDControlFn(turntarget):
    turnkP = 0 #tune
    turnkI = 0 #tune
    turnkD = 0 #tune

    turnderivative = 0
    turnpreverror = 0
    turntotalerror = 0

    while enableturnPID:
        turnerror = turntarget - inertial.heading(DEGREES)

        turnderivative = turnerror - turnpreverror

        turntotalerror += turnerror

        turn_motor_power = (turnerror * turnkP + turnderivative * turnkD + turntotalerror * turnkI)

        left_drive_smart.spin(FORWARD, turn_motor_power, RPM) #fix
        right_drive_smart.spin(REVERSE, turn_motor_power, RPM) #fix

        turnpreverror = turnerror
        wait(50, MSEC)

comp = Competition(user_control, auton)
pre_auton()

'''
brain = Brain()
inertial = Inertial(Ports.PORT8)
controller_1 = Controller(PRIMARY)
inertial.calibrate()
wait(2, SECONDS)
inertial.set_heading(0, DEGREES)
inertial.set_rotation(0, DEGREES)
drivetrain.set_turn_velocity(25, PERCENT)

def inertial_values():
    while True:
        controller_1.screen.clear_screen()
        controller_1.screen.set_cursor(1,1)
        controller_1.screen.print(inertial.heading(DEGREES))
        controller_1.screen.next_row()
        controller_1.screen.print(inertial.rotation(DEGREES))
        drivetrain.turn(RIGHT)
        if inertial.rotation(DEGREES) >= 90:
            drivetrain.stop()
            break
        wait(15, MSEC)

printing = Thread(inertial_values)
'''
