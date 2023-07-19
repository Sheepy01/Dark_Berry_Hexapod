
import sys
import time
import math
import pygame
import loading_animation as load
from colorama import Fore, Style
from adafruit_servokit import ServoKit
from math import sin, cos, tanh, tan, radians, pi
from pyPS4Controller.controller import Controller

kit = ServoKit(channels=16)

#CONSTANTS
BATT_VOLTAGE = 0  # 12V Battery analog voltage input port
RAD_TO_DEG = 57.2957795131

COXA_LENGTH = 51  # leg part lengths
FEMUR_LENGTH = 65
TIBIA_LENGTH = 121

FRAME_TIME_MS = 20  # frame time (20msec = 50Hz)

HOME_X = [82.0, 0.0, -82.0, -82.0, 0.0, 82.0]  # coxa-to-toe home positions
HOME_Y = [82.0, 116.0, 82.0, -82.0, -116.0, -82.0]
HOME_Z = [-110.0, -110.0, -110.0, -110.0, -110.0, -110.0]

BODY_X = [110.4, 0.0, -110.4, -110.4, 0.0, 110.4]  # body center-to-coxa servo distances
BODY_Y = [58.4, 90.8, 58.4, -58.4, -90.8, -58.4]
BODY_Z = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

COXA_CAL = [2, -1, -1, -3, -2, -3]  # servo calibration constants
FEMUR_CAL = [4, -2, 0, -1, 0, 0]
TIBIA_CAL = [0, -3, -3, -2, -3, -1]

MIN_PULSE = 500
MAX_PULSE = 2500
nbPCAServo = 16

temp = 0                             # mode and control variables
mode = 0
gait = 0
gait_speed = 0

L0 = 0.0                             # inverse kinematics variables
L3 = 0.0
gamma_femur = 0.0
phi_tibia = 0.0
phi_femur = 0.0
theta_tibia = 0.0
theta_femur = 0.0
theta_coxa = 0.0

leg_num = 0                          # positioning and walking variables
z_height_LED_color = 0
totalX = 0
totalY = 0
totalZ = 0
tick = 0
duration = 0
numTicks = 0
z_height_left = 0
z_height_right = 0
commandedX = 0
commandedY = 0
commandedR = 0

step_height_multiplier = 0.0
strideX = 0.0
strideY = 0.0
strideR = 0.0
sinRotX = 0.0
sinRotY = 0.0
sinRotZ = 0.0
cosRotX = 0.0
cosRotY = 0.0
cosRotZ = 0.0
rotOffsetX = 0.0
rotOffsetY = 0.0
rotOffsetZ = 0.0
amplitudeX = 0.0
amplitudeY = 0.0
amplitudeZ = 0.0
offset_X = [0.0] * 6
offset_Y = [0.0] * 6
offset_Z = [0.0] * 6
current_X = [0.0] * 6
current_Y = [0.0] * 6
current_Z = [0.0] * 6
capture_offsets = 0

leg1_IK_control = False
leg6_IK_control = False
reset_position = False

tripod_case = [1, 2, 1, 2, 1, 2]     # for tripod gait walking

gamepad_error = 0

previousTime = 0

currentTime = 0

# INITIALIZATION
def setup():

    global reset_position
    global gamepad_error
    global leg_num
    global offset_X
    global offset_Y
    global offset_Z
    global capture_offsets
    global step_height_multiplier
    global mode
    global gait
    global gait_speed
    global reset_position
    global leg1_IK_control
    global leg6_IK_control

    load.loading_screen()

    #INITIALIZE PCA9685
    kit = ServoKit(channels=16)
    for i in range(nbPCAServo):
        kit.servo[i].set_pulse_width_range(MIN_PULSE, MAX_PULSE)
        kit.servo[i].angle = 90
        time.sleep(2)

    pygame.init()
    pygame.joystick.init()
        
    if(pygame.joystick.get_count() == 0):
        print("No controller connected!")
        quit()

    joystick_count = pygame.joystick.get_count()
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()
        print("Joystick", i + 1, ":", joystick.get_name())
    
    if joystick_count > 0:
        gamepad_error = 0
    else:
        gamepad_error = 1

    if gamepad_error == 0:
        print("Controller successfully connected")
    else:
        print("Controller not found")
    
    for leg_num in range(6):
        offset_X[leg_num] = 0.0
        offset_Y[leg_num] = 0.0
        offset_Z[leg_num] = 0.0

    capture_offsets = False
    step_height_multiplier = 2.0

    #mode = 0
    #gait = 0
    gait_speed = 0
    reset_position = True
    leg1_IK_control = True
    leg6_IK_control = True


def main():

    #INTIALIZATION
    setup()
    
    while True:
        global FRAME_TIME_MS
        global HOME_X
        global HOME_Y
        global HOME_Z                          
        global mode
        global gait
        global leg_num 
        global offset_X
        global offset_Y
        global offset_Z
        global current_X
        global current_Y
        global current_Z
        global leg1_IK_control
        global leg6_IK_control
        global reset_position
        global previousTime
        global currentTime
        global previousTime

        currentTime = int(round(time.time() *1000))  # in milliseconds
        if currentTime - previousTime >FRAME_TIME_MS:
            previousTime = currentTime

        # Read controller and process inputs
        process_gamepad()

        # Reset legs to home position when commanded
        if reset_position:
            for leg_num in range(6):
                current_X[leg_num] = HOME_X[leg_num]
                current_Y[leg_num] = HOME_Y[leg_num]
                current_Z[leg_num] = HOME_Z[leg_num]
                reset_position = False

        # Position legs using IK calculations unless set all to 90 degrees mode
        if mode < 99:
            for leg_num in range(0, 6):
                leg_IK(leg_num, current_X[leg_num] + offset_X[leg_num], current_Y[leg_num] + offset_Y[leg_num], current_Z[leg_num] + offset_Z[leg_num])

        # Reset leg lift first pass flags if needed
        if mode != 4:
            leg1_IK_control = True
            leg6_IK_control = True

        # Process modes
        if mode == 1:
            if gait == 0:
                tripod_gait()
        #     elif gait == 1:
        #         wave_gait()
        #     elif gait == 2:
        #         ripple_gait()
        #     elif gait == 3:
        #         tetrapod_gait()
        # elif mode == 2:
        #     translate_control()
        # elif mode == 3:
        #     rotate_control()
        # elif mode == 4:
        #     one_leg_lift()
        # elif mode == 99:
        #     set_all_90()


def map_input(input_value, input_min, input_max, output_min, output_max):
    # Map the input value to the output range
    mapped_value = (input_value - input_min) * (output_max - output_min) // (input_max - input_min) + output_min
    # Constrain the mapped value within the output range
    mapped_value = max(min(mapped_value, output_max), output_min)
    return mapped_value

def process_gamepad():

    #GLOBALS
    global mode
    global gait
    global reset_position
    global gamepad_vibrate
    global gait_speed
    global capture_offsets
    global offset_X
    global offset_Y
    global offset_Z
    global leg1_IK_control
    global leg6_IK_control
    global step_height_multiplier
    global commandedX
    global commandedY
    global commandedR

    #CONSTANTS
    # Initialize variables for maximum and minimum values
    left_joystick_x_max = -1.0
    left_joystick_x_min = 1.0
    left_joystick_y_max = -1.0
    left_joystick_y_min = 1.0

    right_joystick_x_max = -1.0
    right_joystick_x_min = 1.0
    right_joystick_y_max = -1.0
    right_joystick_y_min = 1.0

    DPAD_UP = (0, 1)
    DPAD_DOWN = (0, -1)
    DPAD_LEFT = (-1, 0)
    DPAD_RIGHT = (1, 0)
    BUTTON_X = 0
    BUTTON_CIRCLE = 1
    BUTTON_TRIANGLE = 2
    BUTTON_SQUARE = 3
    L1_IN = 4
    R1_IN = 5
    L2_IN = 6
    R2_IN = 7
    BUTTON_SHARE = 8
    OPTIONS_BUTTON = 9
    PS_BUTTON = 10
    L_STICK_IN = 11
    R_STICK_IN = 12
    TOUCH_PAD_CLICK_BUTTON = 15
    L_STICK_X_AXIS = 0
    L_STICK_Y_AXIS = 1
    R_STICK_X_AXIS = 3
    R_STICK_Y_AXIS = 4

    for event in pygame.event.get():

        if event.type == pygame.JOYBUTTONDOWN:
            button_id = event.button
            button_name = pygame.joystick.Joystick(0).get_button(button_id)
            print(f"Button {button_id} ({button_name}) pressed")

        if event.type == pygame.JOYHATMOTION:
            if event.value == DPAD_UP:
                mode = 0
                gait = 2
                reset_position = True
                print(f"Mode: {mode}")
                print(f"Gait: {gait}")

            if event.value == DPAD_DOWN:
                mode = 0
                gait = 0
                reset_position = True
                print(f"Mode: {mode}")
                print(f"Gait: {gait}")

            if event.value == DPAD_LEFT:
                mode = 0
                gait = 1
                reset_position = True
                print(f"Mode: {mode}")
                print(f"Gait: {gait}")

            if event.value == DPAD_RIGHT:
                mode = 0
                gait = 3
                reset_position = True
                print(f"Mode: {mode}")
                print(f"Gait: {gait}")

        if event.type == pygame.JOYAXISMOTION:
            # Left Joystick X Axis
            if event.axis == L_STICK_X_AXIS:
                L3_x = event.value
                L3_x_result = map_input(L3_x, left_joystick_x_max, left_joystick_x_min, -127, 127)
                commandedY = L3_x_result

            # Left Joystick Y Axis
            elif event.axis == L_STICK_Y_AXIS:
                L3_y = event.value
                L3_y_result = map_input(L3_y, left_joystick_y_max, left_joystick_y_min, -127, 127)
                commandedX = L3_y_result

            # Right Joystick X Axis
            if event.axis == R_STICK_X_AXIS:
                R3_x = event.value
                R3_x_result = map_input(R3_x, right_joystick_x_max, right_joystick_x_min, -127, 127)
                commandedR = R3_x_result

            # Right Joystick Y Axis
            elif event.axis == R_STICK_Y_AXIS:
                R3_y = event.value
                R3_y_result = map_input(R3_y, right_joystick_y_max, right_joystick_y_min, -127, 127)
                commandedR = R3_y_result

        if event.type == pygame.JOYBUTTONDOWN:
            if event.button == BUTTON_TRIANGLE:
                mode = 1
                reset_position = True
                print(f"Mode: {mode}")

            if event.button == BUTTON_SQUARE:
                mode = 2
                reset_position = True
                print(f"Mode: {mode}")

            if event.button == BUTTON_CIRCLE:
                mode = 3
                reset_position = True
                print(f"Mode: {mode}")

            if event.button == BUTTON_X:
                mode = 4
                reset_position = True
                print(f"Mode: {mode}")

            if event.button == BUTTON_SHARE:
                print("BUTTON_SHARE")
            if event.button == PS_BUTTON:
                print("PS_BUTTON")
            if event.button == OPTIONS_BUTTON:
                print("OPTIONS_BUTTON")
            if event.button == L2_IN:
                print("L2_IN")
            if event.button == R2_IN:
                print("R2_IN")
            if event.button == L_STICK_IN:
                print("L_STICK_IN")
            if event.button == R_STICK_IN:
                print("R_STICK_IN")
            if event.button == L1_IN:
                print("LEFT_BUMPER")
            if event.button == R1_IN:
                print("RIGHT_BUMPER")
            if event.button == TOUCH_PAD_CLICK_BUTTON:
                print("TOUCH_PAD_CLICK_BUTTON")

# Leg IK Routine
def leg_IK(leg_number, X, Y, Z):

    # compute target femur-to-toe (L3) length
    L0 = math.sqrt(X**2 + Y**2) - COXA_LENGTH
    L3 = math.sqrt(L0**2 + Z**2)

    # process only if reach is within possible range (not too long or too short!)
    if (L3 < (TIBIA_LENGTH + FEMUR_LENGTH)) and (L3 > (TIBIA_LENGTH - FEMUR_LENGTH)):
        # compute tibia angle
        phi_tibia = math.acos((FEMUR_LENGTH**2 + TIBIA_LENGTH**2 - L3**2) / (2 * FEMUR_LENGTH * TIBIA_LENGTH))
        theta_tibia = phi_tibia * RAD_TO_DEG + 23.0 + TIBIA_CAL[leg_number]
        theta_tibia = max(min(theta_tibia, 180.0), 0.0)

        # compute femur angle
        gamma_femur = math.atan2(Z, L0)
        phi_femur = math.acos((FEMUR_LENGTH**2 + L3**2 - TIBIA_LENGTH**2) / (2 * FEMUR_LENGTH * L3))
        theta_femur = (phi_femur + gamma_femur) * RAD_TO_DEG + 14.0 + 90.0 + FEMUR_CAL[leg_number]
        theta_femur = max(min(theta_femur, 180.0), 0.0)

        # compute coxa angle
        theta_coxa = math.atan2(X, Y) * RAD_TO_DEG + COXA_CAL[leg_number]

        # # output to the appropriate leg
        if leg_number == 0:
            if leg1_IK_control:  # flag for IK or manual control of leg
                theta_coxa += 45.0  # compensate for leg mounting
                theta_coxa = max(min(theta_coxa, 180.0), 0.0)
                print(theta_coxa, theta_femur, theta_tibia)
                kit.servo[0].angle = theta_coxa
                kit.servo[1].angle = theta_femur
                kit.servo[2].angle = theta_tibia
        # if leg_number == 1:
            # theta_coxa += 90.0  # compensate for leg mounting
            # theta_coxa = max(min(theta_coxa, 180.0), 0.0)
            # print(theta_coxa, theta_femur, theta_tibia)
            # kit.servo[0].angle = theta_coxa
            # kit.servo[1].angle = theta_femur
            # kit.servo[2].angle = theta_tibia
        # elif leg_number == 2:
        #     theta_coxa += 135.0  # compensate for leg mounting
        #     theta_coxa = max(min(theta_coxa, 180.0), 0.0)
        #     print(theta_coxa, theta_femur, theta_tibia)
        #     #kit.servo[COXA3_SERVO].angle = theta_coxa
        #     #kit.servo[FEMUR3_SERVO].angle = theta_femur
        #     #kit.servo[TIBIA3_SERVO].angle = theta_tibia
        # if leg_number == 3:
            # if theta_coxa < 0:  # compensate for leg mounting
                # theta_coxa += 225.0  # need to use different positive and negative offsets
            # else:
                # theta_coxa -= 135.0  # due to atan2 results above!
            # theta_coxa = max(min(theta_coxa, 180.0), 0.0)
            # print(theta_coxa, theta_femur, theta_tibia)
            # kit.servo[0].angle = theta_coxa
            # kit.servo[1].angle = theta_femur
            # kit.servo[2].angle = theta_tibia
        # if leg_number == 4:
            # if theta_coxa < 0:  # compensate for leg mounting
                # theta_coxa += 270.0  # need to use different positive and negative offsets
            # else:
                # theta_coxa -= 90.0  # due to atan2 results above!
            # theta_coxa = max(min(theta_coxa, 180.0), 0.0)
            # print(theta_coxa, theta_femur, theta_tibia)
            # kit.servo[0].angle = theta_coxa
            # kit.servo[1].angle = theta_femur
            # kit.servo[2].angle = theta_tibia
        # elif leg_number == 5:
        #     if leg6_IK_control:  # flag for IK or manual control of leg
        #         if theta_coxa < 0:  # compensate for leg mounting
        #             theta_coxa += 315.0  # need to use different positive and negative offsets
        #         else:
        #             theta_coxa -= 45.0  # due to atan2 results above!
        #         theta_coxa = max(min(theta_coxa, 180.0), 0.0)
        #         print(theta_coxa, theta_femur, theta_tibia)
        #         #kit.servo[COXA6_SERVO].angle = theta_coxa
        #         #kit.servo[FEMUR6_SERVO].angle = theta_femur
        #         #kit.servo[TIBIA6_SERVO].angle = theta_tibia


def tripod_gait():

    time.sleep(0.015)

    global commandedX
    global commandedY
    global commandedR
    global RAD_TO_DEG
    global FRAME_TIME_MS
    global HOME_X
    global HOME_Y
    global HOME_Z
    global tick
    global duration
    global numTicks
    global amplitudeX
    global amplitudeY
    global amplitudeZ
    global current_X
    global current_Y
    global current_Z
    global tripod_case

    # If commands are more than the deadband, then process
    if (abs(commandedX) > 15) or (abs(commandedY) > 15) or (abs(commandedR) > 15) or (tick > 0):
        compute_strides()
        numTicks = round(duration / FRAME_TIME_MS / 2.0)  # Total ticks divided into the two cases
        for leg_num in range(6):
            compute_amplitudes(leg_num)
            if tripod_case[leg_num] == 1:  # Move foot forward (raise and lower)
                current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(pi * tick / numTicks)
                current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(pi * tick / numTicks)
                current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(pi * tick / numTicks)
                if tick >= numTicks - 1:
                    tripod_case[leg_num] = 2
            elif tripod_case[leg_num] == 2:  # Move foot back (on the ground)
                current_X[leg_num] = HOME_X[leg_num] + amplitudeX * cos(pi * tick / numTicks)
                current_Y[leg_num] = HOME_Y[leg_num] + amplitudeY * cos(pi * tick / numTicks)
                current_Z[leg_num] = HOME_Z[leg_num]
                if tick >= numTicks - 1:
                    tripod_case[leg_num] = 1

        # Increment tick
        if tick < numTicks - 1:
            tick += 1
        else:
            tick = 0


def constrain(value, minimum, maximum):
    constrained_value = max(minimum, min(value, maximum))
    return constrained_value



# //***********************************************************************
# // Compute walking stride lengths
# //***********************************************************************
def compute_strides():
    global strideX
    global strideY
    global strideR
    global sinRotZ
    global cosRotZ
    global gait_speed
    global duration
    # Compute stride lengths
    strideX = 90 * commandedX / 12
    # print(strideX)
    strideY = 90 * commandedY / 12
    # print(strideY)
    strideR = 35 * commandedR / 12
    # print(strideR)

    # Compute rotation trig
    sinRotZ = sin(radians(strideR))
    # print(sinRotZ)
    cosRotZ = cos(radians(strideR))
    # print(cosRotZ)

    # Set duration for normal and slow speed modes
    if gait_speed == 0:
        duration = 720
    else:
        duration = 2160

# //***********************************************************************
# // Compute walking amplitudes
# //***********************************************************************
def compute_amplitudes(leg_num):
    global totalX
    global totalY
    global rotOffsetX
    global rotOffsetY
    global rotOffsetZ
    global amplitudeX
    global amplitudeY
    global amplitudeZ
    global strideX
    global strideY
    global strideR

    # Compute total distance from center of body to toe
    totalX = HOME_X[leg_num] + BODY_X[leg_num]
    totalY = HOME_Y[leg_num] + BODY_Y[leg_num]

    # Compute rotational offset
    rotOffsetX = totalY * sinRotZ + totalX * cosRotZ - totalX
    rotOffsetY = totalY * cosRotZ - totalX * sinRotZ - totalY

    # Compute X and Y amplitude and constrain to prevent legs from crashing into each other
    amplitudeX = ((strideX + rotOffsetX) / 2.0)
    amplitudeY = ((strideY + rotOffsetY) / 2.0)
    amplitudeX = constrain(amplitudeX, -50, 50)
    # print(amplitudeX)
    amplitudeY = constrain(amplitudeY, -50, 50)
    # print(amplitudeY)

    # Compute Z amplitude
    if abs(strideX + rotOffsetX) > abs(strideY + rotOffsetY):
        amplitudeZ = step_height_multiplier * (strideX + rotOffsetX) / 4.0
        # print(amplitudeZ)
    else:
        amplitudeZ = step_height_multiplier * (strideY + rotOffsetY) / 4.0
        # print(amplitudeZ)	



if __name__ == '__main__':
    main()

