
# Libraries
import time
import math
import pygame
import board
import busio
import cv2
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit
from adafruit_pca9685 import PCA9685
from picamera2 import Picamera2, Preview
from libcamera import controls
from tqdm import tqdm

#CONSTANTS
COXA1_SERVO  = 0 
FEMUR1_SERVO = 1
TIBIA1_SERVO = 2
COXA2_SERVO  = 4
FEMUR2_SERVO = 5
TIBIA2_SERVO = 6
COXA3_SERVO  = 8
FEMUR3_SERVO = 9
TIBIA3_SERVO = 10
COXA4_SERVO  = 0
FEMUR4_SERVO = 1
TIBIA4_SERVO = 2
COXA5_SERVO  = 4
FEMUR5_SERVO = 5
TIBIA5_SERVO = 6
COXA6_SERVO  = 8
FEMUR6_SERVO = 9
TIBIA6_SERVO = 10

ARM_BASE_SERVO = 0
ARM_SHOULDER_SERVO = 1
ARM_ELBOW_SERVO = 2
ARM_WRIST1_SERVO = 3
ARM_WRIST2_SERVO = 4
ARM_GRIPPER_SERVO = 5

RAD_TO_DEG = 57.2957795131
COXA_LENGTH = 51  # leg part lengths
FEMUR_LENGTH = 65
TIBIA_LENGTH = 121
ARM_COXA_LENGTH = 114 # arm part lengths
ARM_FEMUR_LENGTH = 135
ARM_TIBIA_LENGTH = 160
TRAVEL = 40
A12DEG = 209440;           # 12 degrees in radians x 1,000,000
A30DEG = 523599;           # 30 degrees in radians x 1,000,000
FRAME_TIME_MS = 20  # frame time (20msec = 50Hz)
HOME_Z_VALUE = -80
HOME_X = [82.0, 0.0, -82.0, -82.0, 0.0, 82.0]  # coxa-to-toe home positions
HOME_Y = [82.0, 116.0, 82.0, -82.0, -116.0, -82.0]
HOME_Z = [HOME_Z_VALUE] * 6
BODY_X = [110.4, 0.0, -110.4, -110.4, 0.0, 110.4]  # body center-to-coxa servo distances
BODY_Y = [58.4, 90.8, 58.4, -58.4, -90.8, -58.4]
BODY_Z = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

armMode = False
hexapodMode = False
photogrammetry_mode = False

# servo calibration constants
T_CAL_VAL = -30
F_CAL_VAL = 50
COXA_CAL = [-2, -1, -1, 15, 15, 0]  
FEMUR_CAL = [-20, -5, -5, -17, -7, 5]
TIBIA_CAL = [25, 25, 25, 20, 15, 0]

MIN_PULSE = 0
MAX_PULSE = 0
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
armTotalX = 0
armTotalY = 0
armTotalZ = 0
tick = 0
duration = 0
numTicks = 0
z_height_left = 0
z_height_right = 0
commandedX = 0
commandedY = 0
commandedR = 0
translateX = 0
translateY = 0
translateZ = 0

# commanded_arm_y = 0

step_height_value = 1.0
step_height_multiplier = step_height_value
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
previous_X = [0.0] * 6
previous_Y = [0.0] * 6
previous_Z = [0.0] * 6
capture_offsets = 0

leg1_IK_control = False
leg6_IK_control = False
reset_position = False
leg1_coxa = 0 
leg1_femur = 0
leg1_tibia = 0 
leg6_coxa = 0 
leg6_femur = 0
leg6_tibia = 0 

movement = ""
offset_check = ""

tripod_case = [1, 2, 1, 2, 1, 2]     # for tripod gait walking
wave_case = [1, 2, 3, 4, 5, 6]     # for tripod gait walking
ripple_case = [2, 6, 4, 1, 3, 5]     # for tripod gait walking
tetrapod_case = [1, 3, 2, 1, 2, 3]     # for tripod gait walking

arm_home_position = False

current_arm_theta = [90, 0, 0, 0, 90, 90]
final_arm_theta = [90, 0, 0, 0, 90, 90]
arm_home_angles = [90, 0, 0, 0, 90, 90]

gamepad_error = 0

previousTime = 0
currentTime = 0
millisecs = 0.05
deadband = 10
temp = 0
hexapod_start = False
kit1 = None
kit2 = None
kit3 = None
board1_address = 0x40
board2_address = 0x41
board3_address = 0x42
picam2 = None
previewBuild = False
camera_on = False
active_joint = 1
pic_no = 0
radius_size = 1
form_fill = False
video_res = 1080
current_video_res = None
image_captured_count = 0
video_timer = ""
total_time_at_radius = 0
image_video_choice = ""
font = None
small_font = None
title_font = None
screen = None
photogrammetry_menu = False
photogrammetry_start = False
web_cam = False

# # Open a connection to the webcam (usually device 0, but could be different)
# cap = cv2.VideoCapture(2)

# if not cap.isOpened():
#     print("Error: Could not open video device")
#     exit()

# # Set the width and height of the video capture
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GRAY = (50, 50, 50)
DARK_GRAY = (30, 30, 30)

def clear_PCA9685_boards(addresses):
    i2c = busio.I2C(board.SCL, board.SDA)
    pca_boards = [PCA9685(i2c, address=addr) for addr in addresses]
    for pca in pca_boards:
        pca.channels[0].duty_cycle = 0
        
def intializeArm(angles):
	kit3.servo[ARM_BASE_SERVO].angle = angles[0]
	kit3.servo[ARM_SHOULDER_SERVO].angle = angles[1]
	kit3.servo[ARM_ELBOW_SERVO].angle = angles[2]
	kit3.servo[ARM_WRIST1_SERVO].angle = angles[3]
	kit3.servo[ARM_WRIST2_SERVO].angle = angles[4]
	kit3.servo[ARM_GRIPPER_SERVO].angle = angles[5]

def initializePCA9685Boards():
    global kit1
    global kit2
    global kit3
    global MIN_PULSE
    global MAX_PULSE
    # PCA9685 Initialization
    # clear_PCA9685_boards([board1_address, board2_address, board3_address])
    kit1 = ServoKit(channels=16, address=board1_address)
    kit2 = ServoKit(channels=16, address=board2_address)
    kit3 = ServoKit(channels=16, address=board3_address)
    MIN_PULSE = 500
    MAX_PULSE = 2500
    for i in range(0, 4):
        kit1.servo[i].set_pulse_width_range(MIN_PULSE, MAX_PULSE)
        kit2.servo[i].set_pulse_width_range(MIN_PULSE, MAX_PULSE)
        kit3.servo[i].set_pulse_width_range(MIN_PULSE, MAX_PULSE)

def initializeCameraModule():
    global picam2
    # Camera Module Initialization
    picam2 = Picamera2()
    camera_config = picam2.create_still_configuration(
    main={"size": (1920, 1080)}, 
    lores={"size": (1920, 1080)}, 
    display="lores"
    )
    video_config = picam2.create_video_configuration()
    picam2.configure(camera_config)
    picam2.configure(video_config)
    picam2.set_controls({"AfMode":controls.AfModeEnum.Continuous})

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
    global gait_speed
    global reset_position
    global leg1_IK_control
    global leg6_IK_control
    global kit1
    global kit2
    global kit3
    global MIN_PULSE
    global MAX_PULSE
    global offset_check
    global picam2
    global font
    global small_font
    global title_font
    global screen
    
    initializePCA9685Boards()
        
    initializeCameraModule()

    # Initialize the robotic arm to its intial position
    intializeArm(arm_home_angles)
    
    # Hexapod Legs Initialization
    for leg_num in range(6):
        offset_X[leg_num] = 0.0
        offset_Y[leg_num] = 0.0
        offset_Z[leg_num] = 0.0

    capture_offsets = False
    offset_check = "OFF"
    step_height_multiplier = step_height_value

    gait_speed = 1
    reset_position = True
    leg1_IK_control = True
    leg6_IK_control = True


    pygame.init()
    pygame.joystick.init()
    # Set up display
    window_size = (800, 600)
    screen = pygame.display.set_mode(window_size)
    pygame.display.set_caption('Hexapod Information Display')

    # Set up fonts
    font = pygame.font.Font(None, 36)
    small_font = pygame.font.Font(None, 24) 
    title_font = pygame.font.Font(None, 48)

        
    if(pygame.joystick.get_count() == 0):
        print("No controller connected!")
        quit()

    joystick_count = pygame.joystick.get_count()
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()
        print("Joystick", i, ":", joystick.get_name())
    
    if joystick_count > 0:
        gamepad_error = 0
    else:
        gamepad_error = 1

    if gamepad_error == 0:
        print("Controller successfully connected")
        return joystick
    else:
        print("Controller not found")

def draw_text(screen, text, pos, font, color=(255, 255, 255)):
    if not isinstance(text, str):
        text = str(text)
    text_surface = font.render(text, True, color)
    screen.blit(text_surface, pos)

def print_continuous_statement(window, statement):
    height, width = window.getmaxyx()
    x = 0
    window.clear()
    window.addstr(height // 2, max((width - len(statement)) // 2, 0), statement)
    window.refresh()
    x = (x + 1) % width

def determine_display_info():
    global pic_no, radius_size, form_fill, video_res, image_captured_count, current_video_res, video_timer, total_time_at_radius, image_video_choice
    global hexapodMode, armMode, photogrammetry_mode, photogrammetry_menu, mode, gait_speed, gait, step_height_value, camera_on, active_joint, current_arm_theta
    global movement, offset_check, total_time_at_radius

    info = {
        "mode_name": "",
        "speed": "",
        "step_height": "",
        "gait_name": "",
        "camera_on_off": "",
        "arm_gripper_open_close": "",
        "active_joint_name": ""
    }
    
    if mode == 1:
        info["mode_name"] = "Walking Mode"
    elif mode == 2:
        info["mode_name"] = "Translate Mode"
    elif mode == 3:
        info["mode_name"] = "Rotate Mode"
        
    if gait_speed == 0:
        info["speed"] = "2X"
    elif gait_speed == 1:
        info["speed"] = "1.5X"
    elif gait_speed == 2:
        info["speed"] = "Normal"
    elif gait_speed == 3:
        info["speed"] = "-1.5X"
    elif gait_speed == 4:
        info["speed"] = "-2X"
        
    if gait == 0:
        info["gait_name"] = "Tripod gait"
    elif gait == 1:
        info["gait_name"] = "Wave gait"
    elif gait == 2:
        info["gait_name"] = "Ripple gait"
    elif gait == 3:
        info["gait_name"] = "Tetrapod gait"
        
    if step_height_value == 1:
        info["step_height"] = "1"
    elif step_height_value == 2:
        info["step_height"] = "2"
    elif step_height_value == 3:
        info["step_height"] = "3"
    elif step_height_value == 4:
        info["step_height"] = "4"
    elif step_height_value == 5:
        info["step_height"] = "5"
    elif step_height_value == 6:
        info["step_height"] = "6"
    elif step_height_value == 7:
        info["step_height"] = "7"
    
    if camera_on:
        info["camera_on_off"] = "ON"
    else:
        info["camera_on_off"] = "OFF"
        
    if active_joint == 1:
        info["active_joint_name"] = "SHOULDER"
    elif active_joint == 2:
        info["active_joint_name"] = "ELBOW"
    elif active_joint == 3:
        info["active_joint_name"] = "WRIST 1"
        
    if current_arm_theta[-1] > 50:
        info["arm_gripper_open_close"] = "Open"
    elif current_arm_theta[-1] < 20:
        info["arm_gripper_open_close"] = "Closed"
    else:
        info["arm_gripper_open_close"] = "Middle"

    return info

def printInformation(screen, font, small_font, info, hexapodMode, armMode, movement, offset_check):
    global total_time_at_radius
    global photogrammetry_menu
    global photogrammetry_menu
    global image_video_choice
    global form_fill
    global pic_no
    global radius_size
    # global photogrammetry_start
    padding = 10
    margin = 20
    y_offset = margin

    if hexapodMode:
        draw_text(screen, "******HEXAPOD MENU******", (margin, y_offset), title_font)
        y_offset += 50
        draw_text(screen, f"Mode: {info['mode_name']}", (margin, y_offset), font)
        y_offset += 40
        draw_text(screen, f"Gait: {info['gait_name']}", (margin, y_offset), font)
        y_offset += 40
        draw_text(screen, f"Walking Movement: {movement}", (margin, y_offset), font)
        y_offset += 40
        draw_text(screen, f"Gait Speed: {info['speed']}", (margin, y_offset), font)
        y_offset += 40
        draw_text(screen, f"Step Height: {info['step_height']}", (margin, y_offset), font)
        y_offset += 40
        draw_text(screen, f"Offsets: {offset_check}", (margin, y_offset), font)
        
    elif armMode and not photogrammetry_mode:
        draw_text(screen, "******[HEXAPOD+ARM] MENU******", (margin, y_offset), title_font)
        y_offset += 50
        draw_text(screen, '"ARM":', (margin, y_offset), title_font)
        y_offset += 40
        draw_text(screen, f"Active Joint: {info['active_joint_name']}", (margin, y_offset), font)
        y_offset += 40
        draw_text(screen, f"Joint Angles: {current_arm_theta}", (margin, y_offset), font)
        y_offset += 40
        draw_text(screen, f"Camera: {info['camera_on_off']}", (margin, y_offset), font)
        y_offset += 40
        draw_text(screen, f"Arm Gripper: {info['arm_gripper_open_close']}", (margin, y_offset), font)
        y_offset += 40
        draw_text(screen, '"HEXAPOD":', (margin, y_offset), title_font)
        y_offset += 40
        draw_text(screen, f"Mode: {info['mode_name']}", (margin, y_offset), font)
        y_offset += 40
        draw_text(screen, f"Gait: {info['gait_name']}", (margin, y_offset), font)
        y_offset += 40
        draw_text(screen, f"Walking Movement: {movement}", (margin, y_offset), font)
        y_offset += 40
        draw_text(screen, f"Gait Speed: {info['speed']}", (margin, y_offset), font)
        y_offset += 40
        draw_text(screen, f"Step Height: {info['step_height']}", (margin, y_offset), font)
    elif photogrammetry_mode == True and armMode == True:
        if photogrammetry_menu == False:
            if form_fill == False:
                image_video_choice = input("Enter 'i' for image capture and 'v' for video capture: ")
                if image_video_choice.upper() == 'I':
                    pic_no = int(input("Enter the number of pictures to capture: "))
                elif image_video_choice.upper() == 'V':
                    video_res = int(input("Enter 360 or 480 or 720 or 1080: "))
                    current_video_res = video_res
                radius_size = int(input('''Choose any of the radius size:
                    1) Small (1 metre)
                    2) Medium (2 metre)
                    3) Large (3 metre)
                    Choose 1, 2, or 3: '''))
                form_fill = True
                if radius_size == 1:
                    total_time_at_radius = 60000 # in milliseconds
                elif radius_size == 2:
                    total_time_at_radius = 120000
                elif radius_size == 3:
                    total_time_at_radius = 240000
            elif form_fill == True:
                if pic_no != 1 and 1 <= radius_size <= 3:
                    print("")
                    # print(f'''Place the hexapod at approx {radius_size} metre from the target object
                    #       Enter triangle to start series image capturing''')
        elif photogrammetry_menu == True:
            if image_video_choice.upper() == 'I':
                print(f"Number of images captured: {image_captured_count}")
            elif image_video_choice.upper() == 'V':
                print(f'''Current video resolution: {current_video_res}
                    Video duration: {video_timer}''')   
    # elif photogrammetry_mode and armMode:
    #     if not photogrammetry_menu:
    #         if not form_fill:
    #             draw_text(screen, "Enter 'i' for image capture and 'v' for video capture: ", (margin, y_offset), small_font)
    #             image_video_choice = input("Enter 'i' for image capture and 'v' for video capture: ")
    #             if image_video_choice.upper() == 'I':
    #                 pic_no = int(input("Enter the number of pictures to capture: "))
    #             elif image_video_choice.upper() == 'V':
    #                 video_res = int(input("Enter 360 or 480 or 720 or 1080: "))
    #                 current_video_res = video_res
    #             radius_size = int(input('''Choose any of the radius size:
    #                 1) Small (1 metre)
    #                 2) Medium (2 metre)
    #                 3) Large (3 metre)
    #                 Choose 1, 2, or 3: '''))
    #             if radius_size == 1:
    #                 total_time_at_radius = 60000  # in milliseconds
    #             elif radius_size == 2:
    #                 total_time_at_radius = 120000
    #             elif radius_size == 3:
    #                 total_time_at_radius = 240000
    #             form_fill = True
    #         else:
    #             if pic_no != 1 and 1 <= radius_size <= 3:
    #                 draw_text(screen, f"Place the hexapod at approx {radius_size} metre from the target object", (margin, y_offset), small_font)
    #                 y_offset += 40
    #                 draw_text(screen, "Enter triangle to start series image capturing", (margin, y_offset), small_font)
    #     else:
    #         if image_video_choice.upper() == 'I':
    #             draw_text(screen, f"Number of images captured: {image_captured_count}", (margin, y_offset), small_font)
    #         elif image_video_choice.upper() == 'V':
    #             draw_text(screen, f"Current video resolution: {current_video_res}", (margin, y_offset), small_font)
    #             y_offset += 40
    #             draw_text(screen, f"Video duration: {video_timer}", (margin, y_offset), small_font)
    # text = str(text)
    # text_surface = font.render(text, True, (255, 255, 255))
    # screen.blit(text_surface, pos)
    # global pic_no
    # global radius_size
    # global video_res
    # global image_captured_count
    # global current_video_res
    # global video_timer
    # global total_time_at_radius
    # global image_video_choice

    # mode_name = ""
    # speed = ""
    # step_height = ""
    # gait_name = ""
    # step_height = ""
    # camera_on_off = ""
    # arm_gripper_open_close = ""
    # active_joint_name = ""
    
    # if(mode == 1):
    #     mode_name = "Walking Mode"
    # elif(mode == 2):
    #     mode_name = "Translate Mode"
    # elif(mode == 3):
    #     mode_name = "Rotate Mode"
        
    # if(gait_speed == 0):
    #     speed = "2X"
    # elif(gait_speed == 1):
    #     speed = "1.5X"
    # elif(gait_speed == 2):
    #     speed = "Normal"
    # elif(gait_speed == 3):
    #     speed = "-1.5X"
    # elif(gait_speed == 4):
    #     speed = "-2X"
        
    # if(gait == 0):
    #     gait_name = "Tripod gait"
    # elif(gait == 1):
    #     gait_name = "Wave gait"
    # elif(gait == 2):
    #     gait_name = "Ripple gait"
    # elif(gait == 3):
    #     gait_name = "Tetrapod gait"
        
    # if(step_height_value == 1):
    #     step_height = "1"
    # elif(step_height_value == 2):
    #     step_height = "2"
    # elif(step_height_value == 3):
    #     step_height = "3"
    # elif(step_height_value == 4):
    #     step_height = "4"
    # elif(step_height_value == 5):
    #     step_height = "5"
    # elif(step_height_value == 6):
    #     step_height = "6"
    # elif(step_height_value == 7):
    #     step_height = "7"
    
    # if camera_on == True:
    #     camera_on_off = "ON"
    # else:
    #     camera_on_off = "OFF"
        
    # if active_joint == 1:
    #     active_joint_name = "SHOULDER"
    # if active_joint == 2:
    #     active_joint_name = "ELBOW"
    # if active_joint == 3:
    #     active_joint_name = "WRIST 1"
        
    # if current_arm_theta[len(current_arm_theta) - 1] > 50:
    #     arm_gripper_open_close = "Open"
    # elif current_arm_theta[len(current_arm_theta) - 1] < 20:
    #     arm_gripper_open_close = "Closed"
    # else:
    #     arm_gripper_open_close = "Middle"
        
    # os.system('clear')
    
    # if hexapodMode == True:
    #     print(f'''
    #               :%@%-        .*@@=      
    #                 +@%*      =%@*.       
    #                   .*@*--=%%:.         
    #                     =@#%%*            
    #             ....    -@#%%*    ....    
    #           +@@@@#%#@@@@@@@@@@%##@@@@*: 
    #             ....    =@#%%*    ....    
    #                     =@*%%*            
    #                   .*@*--+@%:          
    #                 =@%*      =%@*.       
    #               :%@%-        :#@@=      
    #               :=:            .=- 
    #             \n\n\t\t******HEXAPOD MENU******
    #             1) Mode: {mode_name}
    #             2) Gait: {gait_name}
    #             3) Walking Movement: {movement}
    #             4) Gait Speed: {speed}
    #             5) Step Height: {step_height}
    #             6) Offsets: {offset_check}''')
                
    # elif armMode == True and photogrammetry_mode == False:
    #     print(f'''
    #               :%@%-        .*@@=      
    #                 +@%*      =%@*.       
    #                   .*@*--=%%:.         
    #                     =@#%%*            
    #             ....    -@#%%*    ....    
    #           +@@@@#%#@@@@@@@@@@%##@@@@*: 
    #             ....    =@#%%*    ....    
    #                     =@*%%*            
    #                   .*@*--+@%:          
    #                 =@%*      =%@*.       
    #               :%@%-        :#@@=      
    #               :=:            .=- 
    #             \n\n\t\t******[HEXAPOD+ARM] MENU******
    #             "ARM":
    #             1) Active Joint: {active_joint_name}
    #             2) Joint Angles: {current_arm_theta}
    #             3) Camera: {camera_on_off}
    #             4) Arm Gripper: {arm_gripper_open_close}
                
    #             "HEXAPOD":
    #             1) Mode: {mode_name}
    #             2) Gait: {gait_name}
    #             3) Walking Movement: {movement}
    #             4) Gait Speed: {speed}
    #             5) Step Height: {step_height}''')
    # elif photogrammetry_mode == True and armMode == True:
    #     if photogrammetry_menu == False:
    #         if form_fill == False:
    #             image_video_choice = input("Enter 'i' for image capture and 'v' for video capture: ")
    #             if image_video_choice.upper() == 'I':
    #                 pic_no = int(input("Enter the number of pictures to capture: "))
    #             elif image_video_choice.upper() == 'V':
    #                 video_res = int(input("Enter 360 or 480 or 720 or 1080: "))
    #                 current_video_res = video_res
    #             radius_size = int(input('''Choose any of the radius size:
    #                 1) Small (1 metre)
    #                 2) Medium (2 metre)
    #                 3) Large (3 metre)
    #                 Choose 1, 2, or 3: '''))
    #             if radius_size == 1:
    #                 total_time_at_radius = 60000 # in milliseconds
    #             elif radius_size == 2:
    #                 total_time_at_radius = 120000
    #             elif radius_size == 3:
    #                 total_time_at_radius = 240000
    #             form_fill = True
    #         elif form_fill == True:
    #             if pic_no != 1 and 1 <= radius_size <= 3:
    #                 print(f'''Place the hexapod at approx {radius_size} metre from the target object
    #                       Enter triangle to start series image capturing''')
    #     elif photogrammetry_menu == True:
    #         if image_video_choice.upper() == 'I':
    #             print(f"Number of images captured: {image_captured_count}")
    #         elif image_video_choice.upper() == 'V':
    #             print(f'''Current video resolution: {current_video_res}
    #                 Video duration: {video_timer}''')
        


def hexapodButtonStart(controller):
    global hexapod_start
    global hexapodMode
    if controller is None:
        return
                
    #PS BUTTON STARTUP
    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 10:
                    if hexapod_start == False:
                        #printInformation()
                        hexapod_start = True
                        hexapodMode = True
                        main()
    
def initiateHexapod():
    global hexapod_start
    #loading_animation()
    #INTIALIZATION
    controller = setup()
    hexapodButtonStart(controller)

def startWebCam():
    global cv2
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame")
        exit()
    cv2.imshow('Webcam Feed', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        exit()

def stopWebCam():
    print("Stopped")

def main():
    while hexapod_start:
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
        if currentTime - previousTime > FRAME_TIME_MS:
            previousTime = currentTime
            screen.fill((0, 0, 0))
            # Read controller and process inputs
            process_gamepad()
            if web_cam:
                startWebCam()
            elif not web_cam:
                stopWebCam()
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
                elif gait == 1:
                    wave_gait()
                elif gait == 2:
                    ripple_gait()
                elif gait == 3:
                    tetrapod_gait()
            elif mode == 2:
                translate_control()
            elif mode == 3:
                rotate_control()
            elif mode == 4:
                arm_control()
            elif mode == 5:
                photogrammetryControl()

        # Clear the screen
        screen.fill((0, 0, 0))
        
        # Update the global variables
        # Here you would update your global variables with the actual robot data
        
        # Determine display information based on modes
        info = determine_display_info()
        
        # Print the information
        printInformation(screen, font, small_font, info, hexapodMode, armMode, movement, offset_check)
        
        # Update the display
        pygame.display.flip()

def loading_animation():
    for i in tqdm (range (101), 
        desc="DarkBerry Loadingâ€¦", 
        ascii=False, ncols=75):
        time.sleep(0.04)
    
    print('''         --                  .-:        
        +#@%+              -#@@+:       
         :@@@@+          -%@@@+         
           =%%%*        -%%%*.          
             .-%@#:  .+@@+..            
               .*@@@@@@%-               
                 @*=%-@=                
      .....     :@@%@%@*      .....     
  .+@@@@@@###%@@@@@@@@@@@@@%#%%@@@@@++  
   :==++++=..:--*@@@@@@%---:.-=+++==::  
                 @#+%=@=                
                :@%%@#@*                
               *@@*==+@@%:              
           .+*%%-.     -*@**:           
          +@@@#:         +@@@#:         
        :%@@%-            :#@@@=        
        -+*:                 +*=.  ''')
	
    print("Complete.")

def process_gamepad():
    #printInformation()
    #GLOBALS
    #global commanded_arm_y
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
    global step_height_value
    global commandedX
    global commandedY
    global commandedR
    global translateX
    global translateY
    global translateZ
    global sinRotX
    global cosRotX
    global sinRotY
    global cosRotY
    global sinRotZ
    global cosRotZ
    global temp
    global hexapod_start
    global movement
    global offset_check
    global armMode
    global hexapodMode
    global current_arm_theta
    global camera_on
    global active_joint
    global photogrammetry_mode
    global photogrammetry_menu
    global photogrammetry_start
    global web_cam

    #CONSTANTS
    # Initialize variables for maximum and minimum values
    left_joystick_x_max = 1.0
    left_joystick_x_min = -1.0
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
        if hexapodMode == True:
            #print("Hexapod Mode Gamepad")
            if event.type == pygame.JOYBUTTONDOWN:
                button_id = event.button
                button_name = pygame.joystick.Joystick(0).get_button(button_id)
                # print(f"Button {button_id} ({button_name}) pressed")

            if event.type == pygame.JOYHATMOTION:
                if event.value == DPAD_UP:
                    mode = 0
                    gait = 2
                    reset_position = True
                    # print(f"Mode: {mode}")
                    # print(f"Gait: {gait}")

                if event.value == DPAD_DOWN:
                    mode = 0
                    gait = 0
                    reset_position = True
                    # print(f"Mode: {mode}")
                    # print(f"Gait: {gait}")

                if event.value == DPAD_LEFT:
                    mode = 0
                    gait = 1
                    reset_position = True
                    # print(f"Mode: {mode}")
                    # print(f"Gait: {gait}")

                if event.value == DPAD_RIGHT:
                    mode = 0
                    gait = 3
                    reset_position = True
                    # print(f"Mode: {mode}")
                    # print(f"Gait: {gait}")

            if event.type == pygame.JOYAXISMOTION:
                # Left Joystick X Axis
                if event.axis == L_STICK_X_AXIS:
                    L3_x = event.value
                    if mode == 1:
                        if(L3_x > 0.1 and L3_x <= 1.0):
                            movement = "Right"
                        elif(L3_x < -0.1 and L3_x >= -1.0):
                            movement = "Left"
                        elif(L3_x <= 0.09 and L3_x >= -0.09):
                            movement = "Steady"
                    sinRotX = math.sin((pygame.joystick.Joystick(0).get_axis(R_STICK_X_AXIS) * (A12DEG / 255.0)) / 3000.0)
                    L3_x_result = map_input(L3_x, left_joystick_x_min, left_joystick_x_max, -127, 127)
                    commandedY = L3_x_result
                    
                    sinRotZ = math.sin(math.radians((map_input(L3_x, left_joystick_x_min, left_joystick_x_max, -A30DEG, A30DEG))))
                    cosRotZ = math.cos(math.radians((map_input(L3_x, left_joystick_x_min, left_joystick_x_max, -A30DEG, A30DEG))))
                    #print(sinRotZ, " ", cosRotZ)
                    #temp = map_input(L3_x, left_joystick_x_min, left_joystick_x_max, 0, 255)

                # Left Joystick Y Axis
                elif event.axis == L_STICK_Y_AXIS:
                    L3_y = event.value
                    if mode == 1:
                        if(L3_y > 0.1 and L3_y <= 1.0):
                            movement = "Backward"
                        elif(L3_y < -0.1 and L3_y >= -1.0):
                            movement = "Forward"
                        elif(L3_y <= 0.09 and L3_y >= -0.09):
                            movement = "Steady"
                    L3_y_result = map_input(L3_y, left_joystick_y_min, left_joystick_y_max, -127, 127)
                    commandedX = L3_y_result
                    translateZ = map_input(L3_y, left_joystick_y_min, left_joystick_y_max, 0, 255)
                    if translateZ > 127.0:
                        translateZ = map_input(translateZ, 128.0, 255.0, 0, 2*TRAVEL)
                        translateZ = -1*translateZ
                    else:
                        translateZ = map_input(translateZ, 0.0, 127.0, -2*TRAVEL , 0)
                        translateZ = -1*translateZ
                    #temp = map_input(L3_y, left_joystick_y_min, left_joystick_y_max, 0, 255)

                # Right Joystick X Axis
                if event.axis == R_STICK_X_AXIS:
                    R3_x = event.value
                    if mode == 1:
                        if(R3_x > 0.1 and R3_x < 1.0):
                            movement = "Rotate Clockwise"
                        elif(R3_x < -0.1 and R3_x > -1.0):
                            movement = "Rotate Anti-Clockwise"
                        elif(R3_x <= 0.09 and R3_x >= -0.09):
                            movement = "Steady"
                    sinRotX = math.sin(math.radians((map_input(R3_x, right_joystick_x_min, right_joystick_x_max, A12DEG, -A12DEG))))
                    cosRotX = math.cos(math.radians((map_input(R3_x, right_joystick_x_min, right_joystick_x_max, A12DEG, -A12DEG))))
                    #print(sinRotX, " ", cosRotX)
                    R3_x_result = map_input(R3_x, right_joystick_x_min, right_joystick_x_max, -127, 127)
                    commandedR = R3_x_result
                    translateY = map_input(R3_x, right_joystick_x_min, right_joystick_x_max, 0, 255)
                    if translateY > 127.0:
                        translateY = map_input(translateY, 128.0, 255.0, 0, 2*TRAVEL)
                    else:
                        translateY = map_input(translateY, 0.0, 127.0, -2*TRAVEL, 0)
                    #print(translateY)
                    #temp = map_input(R3_x, right_joystick_x_min, right_joystick_x_max, 0, 255)

                # Right Joystick Y Axis
                elif event.axis == R_STICK_Y_AXIS:
                    R3_y = event.value                
                    translateX = map_input(R3_y, right_joystick_y_max, right_joystick_y_min, 0, 255)
                    if translateX > 127.0:
                        translateX = map_input(translateX, 128.0, 255.0, 0, 2*TRAVEL)
                    else:
                        translateX = map_input(translateX, 0.0, 127.0, -2*TRAVEL, 0)
                    sinRotY = math.sin(math.radians((map_input(R3_y, right_joystick_y_min, right_joystick_y_max, A12DEG, -A12DEG))))
                    cosRotY = math.cos(math.radians((map_input(R3_y, right_joystick_y_min, right_joystick_y_max, A12DEG, -A12DEG))))
                    #print(sinRotY, " ", cosRotY)
                    #temp = map_input(R3_y, right_joystick_y_min, right_joystick_y_max, 0, 255)

            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == BUTTON_TRIANGLE:
                    mode = 1
                    reset_position = True
                    # print(f"Mode: {mode}")

                if event.button == BUTTON_SQUARE:
                    mode = 2
                    reset_position = True
                    # print(f"Mode: {mode}")

                if event.button == BUTTON_CIRCLE:
                    mode = 3
                    reset_position = True
                    # print(f"Mode: {mode}")

                if event.button == BUTTON_X:
                    mode = 4
                    hexapodMode = False
                    armMode = True
                    reset_position = True
                    print("X")
                    #setArmHomePosition()

                if event.button == BUTTON_SHARE:
                    pass
                    #print("BUTTON_SHARE")
            
                if event.button == PS_BUTTON:
                    #print("PS_BUTTON")
                    #printInformation()
                    hexapod_start = False
                    clear_PCA9685_boards([board1_address, board2_address])
                    for i in range(nbPCAServo):
                        kit1.servo[i].set_pulse_width_range(0, 0)
                        kit2.servo[i].set_pulse_width_range(0, 0)
                    GPIO.cleanup()  # Clean up all GPIO settings
                    exit()
                    
                if event.button == L_STICK_IN or event.button == R_STICK_IN:
                    if web_cam == False:
                        web_cam = True
                    elif web_cam == True:
                        web_cam = False

                    #print("Home Position")

                if event.button == OPTIONS_BUTTON:
                    #print("OPTIONS_BUTTON")
                    if(gait_speed == 0):
                        gait_speed = 1
                        #print(gait_speed)
                    elif(gait_speed == 1):
                        gait_speed = 2
                        #print(gait_speed)
                    elif(gait_speed == 2):
                        gait_speed = 3
                        #print(gait_speed)
                    elif(gait_speed == 3):
                        gait_speed = 4
                        #print(gait_speed)
                    elif(gait_speed == 4):
                        gait_speed = 0
                        #print(gait_speed)

                if event.button == L2_IN:
                    if step_height_value == 1:
                        step_height_value = 2
                        step_height_multiplier = step_height_value
                        #print("Step Height Value: ", step_height_value)
                        
                    elif step_height_value == 2:
                        step_height_value = 3
                        step_height_multiplier = step_height_value
                        #print("Step Height Value: ", step_height_value)
                        
                    elif step_height_value == 3:
                        step_height_value = 4
                        step_height_multiplier = step_height_value
                        #print("Step Height Value: ", step_height_value)
                    
                    elif step_height_value == 4:
                        step_height_value = 5
                        step_height_multiplier = step_height_value
                        #print("Step Height Value: ", step_height_value)
                        
                    elif step_height_value == 5:
                        step_height_value = 6
                        step_height_multiplier = step_height_value
                        #print("Step Height Value: ", step_height_value)
                        
                    elif step_height_value == 6:
                        step_height_value = 7
                        step_height_multiplier = step_height_value
                        #print("Step Height Value: ", step_height_value)
                        
                    elif step_height_value == 7:
                        step_height_value = 1
                        step_height_multiplier = step_height_value
                        #print("Step Height Value: ", step_height_value)
                    
                if event.button == R2_IN:
                    pass
                    #print("R2_IN")
                    
                # if event.button == L_STICK_IN:
                    # print("L_STICK_IN")
                    
                if event.button == R_STICK_IN:
                    pass
                    #print("R_STICK_IN")
                    
                if event.button == L1_IN:
                    #print("LEFT_BUMPER")
                    for leg_num in range(0, 6):
                        offset_X[leg_num] = 0
                        offset_Y[leg_num] = 0
                        offset_Z[leg_num] = 0
                    offset_check = "OFF"
                    leg1_IK_control = True
                    leg6_IK_control = True
                    step_height_multiplier = step_height_value
                    
                    
                if event.button == R1_IN:
                    #print("RIGHT_BUMPER")
                    capture_offsets = True
                    offset_check = "ON"
                    
                if event.button == TOUCH_PAD_CLICK_BUTTON:
                    pass
                    #print("TOUCH_PAD_CLICK_BUTTON")
                    
        elif armMode == True:
            angle_increment = 10
            #print("Arm Mode Gamepad")
            if event.type == pygame.JOYHATMOTION:
                if event.value == DPAD_UP:
                    if current_arm_theta[active_joint] < 180:
                        current_arm_theta[active_joint] += angle_increment

                if event.value == DPAD_DOWN:
                    if current_arm_theta[active_joint] > 0:
                        current_arm_theta[active_joint] -= angle_increment

                if event.value == DPAD_LEFT:
                    if current_arm_theta[0] > 0:
                        current_arm_theta[0] -= angle_increment

                if event.value == DPAD_RIGHT:
                    if current_arm_theta[0] < 180:
                        current_arm_theta[0] += angle_increment

            if event.type == pygame.JOYAXISMOTION:
                # Left Joystick X Axis
                if event.axis == L_STICK_X_AXIS:
                    L3_x = event.value
                    if mode == 1:
                        if(L3_x > 0.1 and L3_x <= 1.0):
                            movement = "Right"
                        elif(L3_x < -0.1 and L3_x >= -1.0):
                            movement = "Left"
                        elif(L3_x <= 0.09 and L3_x >= -0.09):
                            movement = "Steady"
                    L3_x_result = map_input(L3_x, left_joystick_x_min, left_joystick_x_max, -127, 127)
                    commandedY = L3_x_result

                # Left Joystick Y Axis
                elif event.axis == L_STICK_Y_AXIS:
                    L3_y = event.value
                    if mode == 1:
                        if(L3_y > 0.1 and L3_y <= 1.0):
                            movement = "Backward"
                        elif(L3_y < -0.1 and L3_y >= -1.0):
                            movement = "Forward"
                        elif(L3_y <= 0.09 and L3_y >= -0.09):
                            movement = "Steady"
                    L3_y_result = map_input(L3_y, left_joystick_y_min, left_joystick_y_max, -127, 127)
                    commandedX = L3_y_result
                    translateZ = map_input(L3_y, left_joystick_y_min, left_joystick_y_max, 0, 255)
                    if translateZ > 127.0:
                        translateZ = map_input(translateZ, 128.0, 255.0, 0, 2*TRAVEL)
                        translateZ = -1*translateZ
                    else:
                        translateZ = map_input(translateZ, 0.0, 127.0, -2*TRAVEL , 0)
                        translateZ = -1*translateZ

                # Right Joystick X Axis
                if event.axis == R_STICK_X_AXIS:
                    R3_x = event.value
                    if mode == 1:
                        if(R3_x > 0.1 and R3_x < 1.0):
                            movement = "Rotate Clockwise"
                        elif(R3_x < -0.1 and R3_x > -1.0):
                            movement = "Rotate Anti-Clockwise"
                        elif(R3_x <= 0.09 and R3_x >= -0.09):
                            movement = "Steady"
                    R3_x_result = map_input(R3_x, right_joystick_x_min, right_joystick_x_max, -127, 127)
                    commandedR = R3_x_result
                    translateY = map_input(R3_x, right_joystick_x_min, right_joystick_x_max, 0, 255)
                    if translateY > 127.0:
                        translateY = map_input(translateY, 128.0, 255.0, 0, 2*TRAVEL)
                    else:
                        translateY = map_input(translateY, 0.0, 127.0, -2*TRAVEL, 0)

                # Right Joystick Y Axis
                elif event.axis == R_STICK_Y_AXIS:
                    R3_y = event.value                
                    translateX = map_input(R3_y, right_joystick_y_max, right_joystick_y_min, 0, 255)
                    if translateX > 127.0:
                        translateX = map_input(translateX, 128.0, 255.0, 0, 2*TRAVEL)
                    else:
                        translateX = map_input(translateX, 0.0, 127.0, -2*TRAVEL, 0)

            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == BUTTON_TRIANGLE:
                    if photogrammetry_mode:
                        print("/_")
                        photogrammetry_menu = True
                        photogrammetry_start = True
                        mode = 5

                if event.button == BUTTON_SQUARE:
                    if camera_on == True:
						# Capture an image
                        image = picam2.capture_array()
						# Save the image to a file
                        from PIL import Image
                        image = Image.fromarray(image)
                        image.save("captured_image.jpg")
                        print("Image Captured")

                if event.button == BUTTON_CIRCLE:
                    pass

                if event.button == BUTTON_X:
                    mode = 1
                    hexapodMode = True
                    armMode = False
                    reset_position = True
                    print("XXXXXXXX")

                if event.button == BUTTON_SHARE:
                    photogrammetry_mode = True
                    hexapodMode = False
                    armMode = True
                    #print("BUTTON_SHARE")
            
                if event.button == PS_BUTTON:
                    #print("PS_BUTTON")
                    #printInformation()
                    hexapod_start = False
                    clear_PCA9685_boards([board1_address, board2_address])
                    for i in range(nbPCAServo):
                        kit1.servo[i].set_pulse_width_range(0, 0)
                        kit2.servo[i].set_pulse_width_range(0, 0)
                        kit3.servo[i].set_pulse_width_range(0, 0)
                    GPIO.cleanup()  # Clean up all GPIO settings
                    exit()
                    
                if event.button == L_STICK_IN or event.button == R_STICK_IN:
                    if camera_on == False:
                        camera_on = True
                    elif camera_on == True:
                        camera_on = False

                if event.button == OPTIONS_BUTTON:
                    if active_joint == 1:
                        active_joint = 2
                    elif active_joint == 2:
                        active_joint = 3
                    elif active_joint == 3:
                        active_joint = 1
                    #print(active_joint)
                    
                if event.button == L1_IN:
                    if current_arm_theta[5] > 0:
                        current_arm_theta[5] -= angle_increment
                
                if event.button == L2_IN:
                    if current_arm_theta[4] > 0:
                        current_arm_theta[4] -= angle_increment
                    
                if event.button == R1_IN:
                    if current_arm_theta[5] < 180:
                        current_arm_theta[5] += angle_increment
                    
                if event.button == R2_IN:
                    if current_arm_theta[4] < 180:
                        current_arm_theta[4] += angle_increment
                    
                if event.button == TOUCH_PAD_CLICK_BUTTON:
                    pass

def setJoystickValues(time_division):
    global commandedY
    global commandedR
    global mode
    global currentTime
    print(time_division)
    currentTime = int(round(time.time() * 1000))
    while currentTime < time_division:
        print("WHILE")
        commandedY = -30
        commandedR = 30
        mode = 1
        print(current_X[0], current_Y[0], current_Z[0])
    commandedY = 0
    commandedR = 0
    mode = 5

def captureImage():
    from PIL import Image
    picam2.start()
    image = picam2.capture_array()
    image = Image.fromarray(image)
    if image.mode == 'RGBA':
        image = image.convert('RGB')
    filename = f"captured_image_{image_captured_count}.jpg"
    image.save(filename)
    print(f"Image Captured: {filename}")
    picam2.stop()

def captureVideo(total_time):
    picam2.start()
    picam2.start_recording('test_video.h264')
    time.sleep(total_time)
    picam2.stop_recording()
    picam2.stop()
    
image_captured_count = 0
image_capture_time = 5000
total_time_image_capture = image_capture_time * pic_no + total_time_at_radius

def photogrammetryControl():
    global mode
    global image_captured_count
    # print("PERFORMING PHOTOGRAMMETRY")
    if image_video_choice.upper() == 'I':
        print("IMAGE")
        time_div = total_time_at_radius / pic_no
        captureImage()
        for x in range(pic_no):
            setJoystickValues(time_div)
            time.sleep(5000)
            captureImage()
            print("Image Captured")
            image_captured_count += 1
            time.sleep(1000)
    elif image_video_choice == 'V':
        pass
        # captureVideo(total_time_at_radius) 

def camera_control():
    global previewBuild
    global camera_on
    
    if camera_on == True:
        if previewBuild == False:
            picam2.start_preview(Preview.QTGL)
            previewBuild = True
        picam2.start()
    elif camera_on == False and previewBuild == True:
        camera_on = False
        previewBuild = False
        if previewBuild == False:
            picam2.stop_preview()
    
def arm_control():
    if gait == 0:
        tripod_gait()
    elif gait == 1:
        wave_gait()
    elif gait == 2:
        ripple_gait()
    elif gait == 3:
        tetrapod_gait()
    camera_control()
    setArmServoPosition(current_arm_theta)

def armGripperRotate(commanded_arm_gripper_rotate_x):
    global current_arm_theta
    current_arm_theta[4] = commanded_arm_gripper_rotate_x

# def armExpand(commanded_arm_y):
    # global current_arm_theta
    # current_arm_theta[1] = commanded_arm_y
    # current_arm_theta[2] = map_input(commanded_arm_y, 0, 180, final_arm_theta[2], 70)
    # current_arm_theta[3] = map_input(commanded_arm_y, 0, 180, final_arm_theta[3], 175)
    
# def armRotate(commanded_arm_R):
    # global current_arm_theta
    # current_arm_theta[0] = commanded_arm_R

def clampAngle(angle):
	return max(min(angle, 180), 0)

def setArmServoPosition(angle):
    base_theta = clampAngle(angle[0])
    shoulder_theta = clampAngle(angle[1])
    elbow_theta = clampAngle(angle[2])
    wrist1_theta = clampAngle(angle[3])
    wrist2_theta = clampAngle(angle[4])
    arm_gripper_theta = clampAngle(angle[5])
    
    #print(base_theta, shoulder_theta, elbow_theta, wrist1_theta, wrist2_theta, arm_gripper_theta)
    
    kit3.servo[ARM_BASE_SERVO].angle = base_theta
    kit3.servo[ARM_SHOULDER_SERVO].angle = shoulder_theta
    kit3.servo[ARM_ELBOW_SERVO].angle = elbow_theta
    kit3.servo[ARM_WRIST1_SERVO].angle = wrist1_theta
    kit3.servo[ARM_WRIST2_SERVO].angle = wrist2_theta
    kit3.servo[ARM_GRIPPER_SERVO].angle = arm_gripper_theta

def setArmHomePosition():
    for x in range(len(current_arm_theta)):
        final_value = final_arm_theta[x]
        current_value = current_arm_theta[x]
        if current_value < final_value:
            while current_value < final_value:
                current_value += 1
                current_arm_theta[x] = current_value
                # Adjust theta value with a delay of 2 milliseconds
                kit3.servo[ARM_BASE_SERVO].angle = final_arm_theta[0]
                kit3.servo[ARM_SHOULDER_SERVO].angle = final_arm_theta[1]
                kit3.servo[ARM_ELBOW_SERVO].angle = final_arm_theta[2]
                kit3.servo[ARM_WRIST1_SERVO].angle = final_arm_theta[3]
                kit3.servo[ARM_WRIST2_SERVO].angle = final_arm_theta[4]
                #print(current_arm_theta[0], current_arm_theta[1], current_arm_theta[2], current_arm_theta[3], current_arm_theta[4], current_arm_theta[5])
                time.sleep(0.2)
        elif current_value > final_value:
            while current_value > final_value:
                current_value -= 1
                current_arm_theta[x] = current_value
                # Adjust theta value with a delay of 2 milliseconds
                kit3.servo[ARM_BASE_SERVO].angle = final_arm_theta[0]
                kit3.servo[ARM_SHOULDER_SERVO].angle = final_arm_theta[1]
                kit3.servo[ARM_ELBOW_SERVO].angle = final_arm_theta[2]
                kit3.servo[ARM_WRIST1_SERVO].angle = final_arm_theta[3]
                kit3.servo[ARM_WRIST2_SERVO].angle = final_arm_theta[4]
                time.sleep(0.008)
                #print(current_arm_theta[0], current_arm_theta[1], current_arm_theta[2], current_arm_theta[3], current_arm_theta[4], current_arm_theta[5])
    print("Done")



# Leg IK Routine
def leg_IK(leg_number, X, Y, Z):
    # compute target femur-to-toe (L3) length
    L0 = math.sqrt(X**2 + Y**2) - COXA_LENGTH
    L3 = math.sqrt(L0**2 + Z**2)

    # process only if reach is within possible range (not too long or too short!)
    if (L3 < (TIBIA_LENGTH + FEMUR_LENGTH)) and (L3 > (TIBIA_LENGTH - FEMUR_LENGTH)):
        # compute tibia angle
        phi_tibia = math.acos((FEMUR_LENGTH**2 + TIBIA_LENGTH**2 - L3**2) / (2 * FEMUR_LENGTH * TIBIA_LENGTH))
        theta_tibia = phi_tibia * RAD_TO_DEG - 11.5 + TIBIA_CAL[leg_number]
        theta_tibia = int(max(min(theta_tibia, 180.0), 0.0))

        # compute femur angle
        gamma_femur = math.atan2(Z, L0)
        phi_femur = math.acos((FEMUR_LENGTH**2 + L3**2 - TIBIA_LENGTH**2) / (2 * FEMUR_LENGTH * L3))
        theta_femur = (phi_femur + gamma_femur) * RAD_TO_DEG + 14.0 + 90.0 + FEMUR_CAL[leg_number]
        theta_femur = int(max(min(theta_femur, 180.0), 0.0))

        # compute coxa angle
        theta_coxa = math.atan2(X, Y) * RAD_TO_DEG + COXA_CAL[leg_number]

        setServoPosition(leg_number, theta_coxa, theta_femur, theta_tibia) 
    
def setServoPosition(leg_number, theta_coxa, theta_femur, theta_tibia):
    # output to the appropriate leg
    if leg_number == 0:
        if leg1_IK_control:  # flag for IK or manual control of leg
            theta_coxa += 45.0  # compensate for leg mounting
            theta_coxa = int(max(min(theta_coxa, 180.0), 0.0))
            #print(theta_coxa, theta_femur, theta_tibia)
            kit1.servo[COXA1_SERVO].angle = theta_coxa
            kit1.servo[FEMUR1_SERVO].angle = theta_femur
            kit1.servo[TIBIA1_SERVO].angle = theta_tibia
            #print(theta_coxa, theta_femur, theta_tibia)

    elif leg_number == 1:
        theta_coxa += 90.0  # compensate for leg mounting
        theta_coxa = int(max(min(theta_coxa, 180.0), 0.0))
        #print(theta_coxa, theta_femur, theta_tibia)
        kit1.servo[COXA2_SERVO].angle = theta_coxa
        kit1.servo[FEMUR2_SERVO].angle = theta_femur
        kit1.servo[TIBIA2_SERVO].angle = theta_tibia

    elif leg_number == 2:
        theta_coxa += 135.0  # compensate for leg mounting
        theta_coxa = int(max(min(theta_coxa, 180.0), 0.0))
        #print(theta_coxa, theta_femur, theta_tibia)
        kit1.servo[COXA3_SERVO].angle = theta_coxa
        kit1.servo[FEMUR3_SERVO].angle = theta_femur
        kit1.servo[TIBIA3_SERVO].angle = theta_tibia

    elif leg_number == 3:
        if theta_coxa < 0:  # compensate for leg mounting
            theta_coxa += 225.0  # need to use different positive and negative offsets
        else:
            theta_coxa -= 135.0  # due to atan2 results above!
        theta_coxa = int(max(min(theta_coxa, 180.0), 0.0))
        #print(theta_coxa, theta_femur, theta_tibia)
        kit2.servo[COXA4_SERVO].angle = theta_coxa
        kit2.servo[FEMUR4_SERVO].angle = theta_femur
        kit2.servo[TIBIA4_SERVO].angle = theta_tibia

    elif leg_number == 4:
        if theta_coxa < 0:  # compensate for leg mounting
            theta_coxa += 270.0  # need to use different positive and negative offsets
        else:
            theta_coxa -= 90.0  # due to atan2 results above!
        theta_coxa = int(max(min(theta_coxa, 180.0), 0.0))
        #print(theta_coxa, theta_femur, theta_tibia)
        kit2.servo[COXA5_SERVO].angle = theta_coxa
        kit2.servo[FEMUR5_SERVO].angle = theta_femur
        kit2.servo[TIBIA5_SERVO].angle = theta_tibia

    elif leg_number == 5:
        if leg6_IK_control:  # flag for IK or manual control of leg
            if theta_coxa < 0:  # compensate for leg mounting
                theta_coxa += 315.0  # need to use different positive and negative offsets
            else:
                theta_coxa -= 45.0  # due to atan2 results above!
            theta_coxa = int(max(min(theta_coxa, 180.0), 0.0))
            #print(theta_coxa, theta_femur, theta_tibia)
            kit2.servo[COXA6_SERVO].angle = theta_coxa
            kit2.servo[FEMUR6_SERVO].angle = theta_femur
            kit2.servo[TIBIA6_SERVO].angle = theta_tibia

def map_input(input_value, input_min, input_max, output_min, output_max):
    # Map the input value to the output range
    mapped_value = (input_value - input_min) * (output_max - output_min) // (input_max - input_min) + output_min
    # Constrain the mapped value within the output range
    mapped_value = max(min(mapped_value, output_max), output_min)
    return mapped_value

def home_position():
    for leg_num in range(0, 6):
        offset_X[leg_num] = 0
        offset_Y[leg_num] = 0
        offset_Z[leg_num] = 0
        current_X[leg_num] = 0
        current_Y[leg_num] = 0
        current_Z[leg_num] = 0

    for leg_num in range(0, 6):
        current_X[leg_num] = HOME_X[leg_num]
        current_Y[leg_num] = HOME_Y[leg_num]
        current_Z[leg_num] = HOME_Z[leg_num]

def tripod_gait():
    # print("Inside Tripod Gait")
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
    if (abs(commandedX) > deadband) or (abs(commandedY) > deadband) or (abs(commandedR) > deadband) or (tick > 0):
        compute_strides()
        numTicks = round(duration / FRAME_TIME_MS / 2.0)  # Total ticks divided into the two cases
        for leg_num in range(6):
            compute_amplitudes(leg_num)
            if tripod_case[leg_num] == 1:  # Move foot forward (raise and lower)
                current_X[leg_num] = HOME_X[leg_num] - amplitudeX * math.cos(math.pi * tick / numTicks)
                current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * math.cos(math.pi * tick / numTicks)
                current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * math.sin(math.pi * tick / numTicks)
                #print(current_X[0], " ", current_Y[0], " ", current_Z[0])
                if tick >= numTicks - 1:
                    tripod_case[leg_num] = 2
            elif tripod_case[leg_num] == 2:  # Move foot back (on the ground)
                current_X[leg_num] = HOME_X[leg_num] + amplitudeX * math.cos(math.pi * tick / numTicks)
                current_Y[leg_num] = HOME_Y[leg_num] + amplitudeY * math.cos(math.pi * tick / numTicks)
                current_Z[leg_num] = HOME_Z[leg_num]
                if tick >= numTicks - 1:
                    tripod_case[leg_num] = 1

        # Increment tick
        if tick < numTicks - 1:
            tick += 1
        else:
            tick = 0

        print(int(current_X[0]), int(current_Y[0]), int(current_Z[0]))
    # time.sleep(millisecs)

# //***********************************************************************
# // Wave Gait
# // Legs move forward one at a time while the other 5 legs provide support
# //***********************************************************************
def wave_gait():
    #print("Inside Wave Gait")
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
    
    if(abs(commandedX) > deadband or abs(commandedY > deadband) or abs(commandedR) > deadband or tick > 0):
        compute_strides()
        numTicks = round(duration/ FRAME_TIME_MS/ 6.0)
        for leg_num in range(0, 6):
            compute_amplitudes(leg_num)
            if wave_case[leg_num] == 1:
                current_X[leg_num] = HOME_X[leg_num] - amplitudeX * math.cos(math.pi * tick / numTicks)
                current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * math.cos(math.pi * tick/numTicks)
                current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * math.sin(math.pi * tick / numTicks)
                if tick >= numTicks - 1:
                    wave_case[leg_num] = 6

            elif wave_case[leg_num] == 2:
                current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5
                current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5
                current_Z[leg_num] = HOME_Z[leg_num]
                if tick >= numTicks - 1:
                    wave_case[leg_num] = 1
            
            elif wave_case[leg_num] == 3:
                current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5
                current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5
                current_Z[leg_num] = HOME_Z[leg_num]
                if tick >= numTicks - 1:
                    wave_case[leg_num] = 2

            elif wave_case[leg_num] == 4:
                current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5
                current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5
                current_Z[leg_num] = HOME_Z[leg_num]
                if tick >= numTicks - 1:
                    wave_case[leg_num] = 3

            elif wave_case[leg_num] == 5:
                current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5
                current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5
                current_Z[leg_num] = HOME_Z[leg_num]
                if tick >= numTicks - 1:
                    wave_case[leg_num] = 4

            elif wave_case[leg_num] == 6:
                current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5
                current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5
                current_Z[leg_num] = HOME_Z[leg_num]
                if tick >= numTicks - 1:
                    wave_case[leg_num] = 5

    if tick < numTicks - 1:
        tick += 1
    else:
        tick = 0

# //***********************************************************************
# // Ripple Gait
# // Left legs move forward rear-to-front while right also do the same,
# // but right side is offset so RR starts midway through the LM stroke
# //***********************************************************************
def ripple_gait():
    #print("Inside ripple Gait")
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

    if abs(commandedX) > deadband or abs(commandedY) > deadband or abs(commandedR) > deadband or tick > 0:
        compute_strides()
        numTicks = round(duration / FRAME_TIME_MS / 6.0)
        for leg_num in range(0, 6):
            compute_amplitudes(leg_num)
            if ripple_case[leg_num] == 1:
                current_X[leg_num] = HOME_X[leg_num] - amplitudeX * math.cos(math.pi * tick / (numTicks * 2))
                current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * math.cos(math.pi * tick / (numTicks * 2))
                current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * math.sin(math.pi * tick / (numTicks * 2))
                if tick >= numTicks - 1:
                    ripple_case[leg_num] = 2
            
            elif ripple_case[leg_num] == 2:
                current_X[leg_num] = HOME_X[leg_num] - amplitudeX * math.cos(math.pi * (numTicks + tick) / (numTicks * 2))
                current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * math.cos(math.pi * (numTicks + tick) / (numTicks * 2))
                current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * math.sin(math.pi * (numTicks + tick) / (numTicks * 2))
                if tick >= numTicks - 1:
                    ripple_case[leg_num] = 3

            elif ripple_case[leg_num] == 3:
                current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.0
                current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.0
                current_Z[leg_num] = HOME_Z[leg_num]
                if tick >= numTicks - 1:
                    ripple_case[leg_num] = 4

            elif ripple_case[leg_num] == 4:
                current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.0
                current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.0
                current_Z[leg_num] = HOME_Z[leg_num]
                if tick >= numTicks - 1:
                    ripple_case[leg_num] = 5

            elif ripple_case[leg_num] == 5:
                current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.0
                current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.0
                current_Z[leg_num] = HOME_Z[leg_num]
                if tick >= numTicks - 1:
                    ripple_case[leg_num] = 6

            elif ripple_case[leg_num] == 6:
                current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.0
                current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.0
                current_Z[leg_num] = HOME_Z[leg_num]
                if tick >= numTicks - 1:
                    ripple_case[leg_num] = 1

        if tick < numTicks - 1:
            tick += 1
        else:
            tick = 0



# //***********************************************************************
# // Tetrapod Gait
# // Right front and left rear legs move forward together, then right  
# // rear and left middle, and finally right middle and left front.
# //***********************************************************************
def tetrapod_gait():
    #print("Inside Tetrapod Gait")
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

    if abs(commandedX) > deadband or abs(commandedY) > deadband or abs(commandedR) > deadband or tick > 0:
        compute_strides()
        numTicks = round(duration / FRAME_TIME_MS / 3.0)
        for leg_num in range(0, 6):
            compute_amplitudes(leg_num)
            if tetrapod_case[leg_num] == 1:
                current_X[leg_num] = HOME_X[leg_num] - amplitudeX * math.cos(math.pi * tick / numTicks)
                current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * math.cos(math.pi * tick / numTicks)
                current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * math.sin(math.pi * tick / numTicks)
                if tick >= numTicks - 1:
                    tetrapod_case[leg_num] = 2

            elif tetrapod_case[leg_num] == 2:
                current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks
                current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks
                current_Z[leg_num] = HOME_Z[leg_num]
                if tick >= numTicks - 1:
                    tetrapod_case[leg_num] = 3

            elif tetrapod_case[leg_num] == 3:
                current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks
                current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks
                current_Z[leg_num] = HOME_Z[leg_num]
                if tick >= numTicks - 1:
                    tetrapod_case[leg_num] = 1

        if tick < numTicks - 1:
            tick += 1
        else:
            tick = 0



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
    strideX = 90 * commandedX / 127
    strideY = 90 * commandedY / 127
    strideR = 35 * commandedR / 127

    # Compute rotation trig
    sinRotZ = math.sin(math.radians(strideR))
    cosRotZ = math.cos(math.radians(strideR))

    # Set duration for normal and slow speed modes
    if gait_speed == 0:
        duration = 360
    elif(gait_speed == 1):
        duration = 540
    elif(gait_speed == 2):
        duration = 720
    elif(gait_speed == 3):
        duration = 1440
    elif(gait_speed == 4):
        duration = 2160


def constrain(value, minimum, maximum):
    constrained_value = max(minimum, min(value, maximum))
    return constrained_value


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
    amplitudeY = constrain(amplitudeY, -50, 50)

    # Compute Z amplitude
    if abs(strideX + rotOffsetX) > abs(strideY + rotOffsetY):
        amplitudeZ = step_height_multiplier * (strideX + rotOffsetX) / 4.0
    else:
        amplitudeZ = step_height_multiplier * (strideY + rotOffsetY) / 4.0



# //***********************************************************************
# // Body translate with controller (xyz axes)
# //***********************************************************************
def translate_control():
    # print("Inside Translate Control")
    global current_X
    global current_Y
    global current_Z
    global HOME_X
    global HOME_Y
    global HOME_Z
    global translateX
    global translateY
    global translateZ
    global offset_X
    global offset_Y
    global offset_Z
    global capture_offsets
    global mode
    global temp
    global movement

    # Compute X direction move
    if translateX != 0.0:
        if(mode == 2):
            if(translateX > 3):
                movement = "Translate Y Backward"
            elif(translateX < -3):
                movement = "Translate Y Forward"
            elif(translateX <= 2 and translateX >= -2):
                movement = "Steady"
        for leg_num in range(0, 6):
            current_X[leg_num] = HOME_X[leg_num] + translateX

    # Compute Y direction move
    if translateY != 0.0:
        if(mode == 2):
            if(translateY > 3):
                movement = "Translate X Left"
            elif(translateY < -3):
                movement = "Translate X Right"
            elif(translateY <= 2 and translateY >= -2):
                movement = "Steady"
        for leg_num in range(0, 6):
            current_Y[leg_num] = HOME_Y[leg_num] + translateY

    # Compute Z direction move
    if translateZ != 0.0:
        if(mode == 2):
            if(translateZ > 3):
                movement = "Translate Z Down"
            elif(translateZ < -3):
                movement = "Translate Z Up"
            elif(translateZ <= 2 and translateZ >= -2):
                movement = "Steady"
        for leg_num in range(0, 6):
            current_Z[leg_num] = HOME_Z[leg_num] + translateZ

    if (capture_offsets == True):
        for leg_num in range(0, 6):
            offset_X[leg_num] = offset_X[leg_num] + translateX
            offset_Y[leg_num] = offset_Y[leg_num] + translateY
            offset_Z[leg_num] = offset_Z[leg_num] + translateZ
            current_X[leg_num] = HOME_X[leg_num]
            current_Y[leg_num] = HOME_Y[leg_num]
            current_Z[leg_num] = HOME_Z[leg_num]

    if capture_offsets == True:
        capture_offsets = False
        mode = 0

# //***********************************************************************
# // Body rotate with controller (xyz axes)
# //***********************************************************************
def rotate_control():
    global totalX
    global totalY
    global totalZ
    global rotOffsetX
    global rotOffsetY
    global rotOffsetZ
    global current_X
    global current_Y
    global current_Z
    global HOME_X
    global HOME_Y
    global HOME_Z
    global BODY_X
    global BODY_Y
    global BODY_Z
    global translateX
    global translateY
    global translateZ
    global offset_X
    global offset_Y
    global offset_Z
    global capture_offsets
    global mode
    global movement

    L_STICK_X_AXIS = 0
    R_STICK_X_AXIS = 3
    R_STICK_Y_AXIS = 4
    
    sinRotX = math.sin((pygame.joystick.Joystick(0).get_axis(R_STICK_X_AXIS) * (A12DEG / 255.0)) / 3000.0)
    cosRotX = math.cos((pygame.joystick.Joystick(0).get_axis(R_STICK_X_AXIS) * (A12DEG / 255.0)) / 3000.0)
    sinRotY = math.sin((pygame.joystick.Joystick(0).get_axis(R_STICK_Y_AXIS) * (A12DEG / 255.0)) / 3000.0)
    cosRotY = math.cos((pygame.joystick.Joystick(0).get_axis(R_STICK_Y_AXIS) * (A12DEG / 255.0)) / 3000.0)
    sinRotZ = math.sin((pygame.joystick.Joystick(0).get_axis(L_STICK_X_AXIS) * (A30DEG / 255.0)) / 5000.0)
    cosRotZ = math.cos((pygame.joystick.Joystick(0).get_axis(L_STICK_X_AXIS) * (A30DEG / 255.0)) / 5000.0)
    #print(f"Sin X: {sinRotX}, Cos X: {cosRotX}, Sin Y: {sinRotY}, Cos Y: {cosRotY}, Sin Z: {sinRotZ}, Cos Z: {cosRotZ}")
    
    if(mode == 3):
        if(sinRotZ >= 0.009):
            movement = "Rotate Z Clockwise"
        elif(sinRotZ <= -0.009):
            movement = "Rotate Z Anti-Clockwise"
        elif(sinRotY >= 0.009):
            movement = "Rotate X Down"
        elif(sinRotY <= -0.009):
            movement = "Rotate X Up"
        elif(sinRotX >= 0.009):
            movement = "Rotate Y Down"
        elif(sinRotX <= -0.009):
            movement = "Rotate Y Up"
        else:
            movement = "Steady"

    for leg_num in range(0, 6):
        totalX = HOME_X[leg_num] + BODY_X[leg_num]
        totalY = HOME_Y[leg_num] + BODY_Y[leg_num]
        totalZ = HOME_Z[leg_num] + BODY_Z[leg_num]

        # perform 3 axis rotations
        rotOffsetX =  totalX*cosRotY*cosRotZ + totalY*sinRotX*sinRotY*cosRotZ + totalY*cosRotX*sinRotZ - totalZ*cosRotX*sinRotY*cosRotZ + totalZ*sinRotX*sinRotZ - totalX;
        rotOffsetY = -totalX*cosRotY*sinRotZ - totalY*sinRotX*sinRotY*sinRotZ + totalY*cosRotX*cosRotZ + totalZ*cosRotX*sinRotY*sinRotZ + totalZ*sinRotX*cosRotZ - totalY;
        rotOffsetZ =  totalX*sinRotY - totalY*sinRotX*cosRotY + totalZ*cosRotX*cosRotY - totalZ
        #print(rotOffsetX, " ", rotOffsetY, " ", rotOffsetZ)

        # Calculate foot positions to achieve desired rotation
        current_X[leg_num] = HOME_X[leg_num] + rotOffsetX;
        current_Y[leg_num] = HOME_Y[leg_num] + rotOffsetY;
        current_Z[leg_num] = HOME_Z[leg_num] + rotOffsetZ + translateZ;
        #print(current_X[0], " ", current_Y[0], " ", current_Z[0])

        # lock in offsets if commanded
        if capture_offsets == True:
            for leg_num in range(0, 6):
                offset_X[leg_num] = offset_X[leg_num] + rotOffsetX
                offset_Y[leg_num] = offset_Y[leg_num] + rotOffsetY
                offset_Z[leg_num] = offset_Z[leg_num] + rotOffsetZ + translateZ
                current_X[leg_num] = HOME_X[leg_num]
                current_Y[leg_num] = HOME_Y[leg_num]
                current_Z[leg_num] = HOME_Z[leg_num]
        
    # if offsets were commanded, exit current mode
    if capture_offsets == True:
        capture_offsets = False
        mode = 0
    

# Function to read and print current servo positions
def read_servo_positions(leg1 = False, leg6 = False, pin_num = None):
    if leg1 == True:
        #change to kit1
        servo_angle = kit1.servo[pin_num].angle
    if leg6 == True:
        #change to kit2
        servo_angle = kit2.servo[pin_num].angle
    return servo_angle

# //***********************************************************************
# // One leg lift mode
# // also can set z step height using capture offsets
# //***********************************************************************
def one_leg_lift():
    pass
    # global leg1_coxa
    # global leg1_femur
    # global leg1_tibia
    # global leg6_coxa
    # global leg6_femur
    # global leg6_tibia
    # global leg1_IK_control
    # global leg6_IK_control
    # global temp
    # global z_height_right
    # global z_height_left
    # global step_height_multiplier

    # # read current leg 1 servo positions the first time
    # if leg1_IK_control == True:
        # leg1_coxa = read_servo_positions(leg1=True, pin_num=0)
        # leg1_femur = read_servo_positions(leg1=True, pin_num=1)
        # leg1_tibia = read_servo_positions(leg1=True, pin_num=2)
        # leg1_IK_control = False

    # # read current leg 1 servo positions the first time
    # # change pin number if required
    # if leg6_IK_control == True:
        # leg6_coxa = read_servo_positions(leg2=True, pin_num=0)
        # leg6_femur = read_servo_positions(leg2=True, pin_num=1)
        # leg6_tibia = read_servo_positions(leg2=True, pin_num=2)
        # leg6_IK_control = False

    # # process right joystick left/right axis
    # temp = map_input(temp, 0, 255, 45, -45)
    # constrained_coxa1_servo = constrain(int(leg1_coxa + temp), 45, 135)
    # kit1.servo[COXA1_SERVO].angle = constrained_coxa1_servo

    # # process right joystick up/down axis
    # if temp < 117:
        # temp = map_input(temp, 116, 0, 0, 24)
        # constrained_femur1_servo = constrain(int(leg1_femur + temp), 0, 170)
        # kit1.servo[FEMUR1_SERVO].angle = constrained_femur1_servo
        # constrained_tibia1_servo = constrain(int(leg1_tibia + temp), 0, 170)
        # kit1.servo[TIBIA1_SERVO].angle = constrained_tibia1_servo
    # else:
        # z_height_right = constrain(temp, 140, 255)
        # z_height_right = map_input(z_height_right, 140, 255, 1, 8)

    # # Process left joystick left/right axis
    # temp = map_input(temp, 0, 255, 45, -45)
    # constrained_coxa6_servo = constrain(int(leg6_coxa + temp), 45, 135)
    # kit1.servo[COXA6_SERVO].angle = constrained_coxa6_servo

    # # process right joystick up/down axis
    # if temp < 117:
        # temp = map_input(temp, 116, 0, 0, 24)
        # constrained_femur6_servo = constrain(int(leg6_femur + temp), 0, 170)
        # kit1.servo[FEMUR1_SERVO].angle = constrained_femur6_servo
        # constrained_tibia6_servo = constrain(int(leg6_tibia + temp), 0, 170)
        # kit1.servo[TIBIA6_SERVO].angle = constrained_tibia6_servo
    # else:
        # z_height_left = constrain(temp, 140, 255)
        # z_height_left = map_input(z_height_left, 140, 255, 1, 8)
    
    # # process z height adjustment
    # if z_height_left > z_height_right:
        # z_height_right = z_height_left
    # if capture_offsets == True:
        # step_height_multiplier = 2.0 + ((z_height_right - 2.0) / 3.0)
        # capture_offsets = False

if __name__ == '__main__':
    initiateHexapod()
    
