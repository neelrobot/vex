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
arm = Motor(Ports.PORT9, GearSetting.RATIO_18_1, False)
intake.set_velocity(100, PERCENT) #Make sure intake always runs at full speed
enableturnPID = False #Condition for turn PID loop
enablePID = False
goal_clamp = DigitalOut(brain.three_wire_port.h)
macro_speed_factor = 1
goal_clamp_clamped = False

#6-motor drivetrain initialization
left_motor_a = Motor(Ports.PORT1, GearSetting.RATIO_6_1, True)
left_motor_b = Motor(Ports.PORT2, GearSetting.RATIO_6_1, True)
left_motor_c = Motor(Ports.PORT3, GearSetting.RATIO_6_1, True)
left_drive_smart = MotorGroup(left_motor_a, left_motor_b, left_motor_c)
right_motor_a = Motor(Ports.PORT4, GearSetting.RATIO_6_1, False)
right_motor_b = Motor(Ports.PORT5, GearSetting.RATIO_6_1, False)
right_motor_c = Motor(Ports.PORT6, GearSetting.RATIO_6_1, False)
right_drive_smart = MotorGroup(right_motor_a, right_motor_b, right_motor_c)
drivetrain = DriveTrain(left_drive_smart, right_drive_smart, 10, 12.75, 10.625, INCHES, 0.75)


def input_monitoring():
   drivetrain_l_needs_to_be_stopped_controller_1 = False
   drivetrain_r_needs_to_be_stopped_controller_1 = False
   intake_stopped = True
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
               left_drive_smart.set_velocity(drivetrain_left_side_speed/macro_speed_factor, PERCENT)
               left_drive_smart.spin(FORWARD)
           # only tell the right drive motor to spin if the values are not in the deadband range
           if drivetrain_r_needs_to_be_stopped_controller_1:
               right_drive_smart.set_velocity(drivetrain_right_side_speed/macro_speed_factor, PERCENT)
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

           controller_1.buttonL1.pressed(goal_clamper)

           #drive speed macros
           controller_1.buttonL2.pressed(speed_factor_down)
           controller_1.buttonUp.pressed(reset_speed_factor)

           #arm macros
           controller_1.buttonDown.pressed(arm_set_zero)
           controller_1.buttonA.pressed(arm_set)
           controller_1.buttonB.pressed(arm_goal)
           controller_1.buttonY.pressed(arm_reset)
           controller_1.buttonLeft.pressed(arm_failsafe)
           
remote_control_code_enabled = True
control_loop = Thread(input_monitoring)

def pre_auton():
    #inertial.calibrate()
    #wait(2, SECONDS)
    #inertial.set_rotation(0, DEGREES)
    intake.set_velocity(100, PERCENT) #Make sure intake always runs at full speed 

def auton():
    pre_auton()
    intake_chain.spin_for(FORWARD, 1.5, SECONDS)
    intake_chain.spin_for(REVERSE, 5, TURNS)
    intake.stop()

def user_control():
    user_control_loop = Thread(input_monitoring)

def arm_set_zero():
    arm.set_position(0, DEGREES)

def arm_set():
    arm.set_velocity(25, PERCENT)
    arm.spin_to_position(145, DEGREES)

def arm_goal():
    arm.set_velocity(25, PERCENT)
    arm.spin_to_position(840, DEGREES)

def arm_reset():
    arm.set_velocity(50, PERCENT)
    arm.spin_to_position(0, DEGREES)

def arm_failsafe():
    arm.spin(REVERSE)
    wait(1, SECONDS)
    arm.stop()

def goal_clamper():
    global goal_clamp_clamped
    if not goal_clamp_clamped:
        goal_clamp.set(True)
        goal_clamp_clamped = True
        wait(15, MSEC)
    elif goal_clamp_clamped:
        goal_clamp.set(False)
        goal_clamp_clamped = False
        wait(15, MSEC)

def speed_factor_down():
    global macro_speed_factor
    macro_speed_factor += 3

def reset_speed_factor():
    global macro_speed_factor
    macro_speed_factor = 1

def temp_monitor():
    while True:
        brain.screen.clear_screen()
        brain.screen.set_cursor(1,1)
        brain.screen.print(left_motor_a.temperature(TemperatureUnits.FAHRENHEIT))
        brain.screen.next_row()
        brain.screen.print(left_motor_b.temperature(TemperatureUnits.FAHRENHEIT))
        brain.screen.next_row()
        brain.screen.print(left_motor_c.temperature(TemperatureUnits.FAHRENHEIT))
        brain.screen.next_row()
        brain.screen.print(right_motor_a.temperature(TemperatureUnits.FAHRENHEIT))
        brain.screen.next_row()
        brain.screen.print(right_motor_b.temperature(TemperatureUnits.FAHRENHEIT))
        brain.screen.next_row()
        brain.screen.print(right_motor_c.temperature(TemperatureUnits.FAHRENHEIT))
        wait(5, SECONDS)

motor_temp = Thread(temp_monitor)

#Function for lateral PID, needs thread
def PIDControlFn(target):
    kP = 0.01 #tune
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
