

import pygame

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


# Map the input value to the output range
def map_input(input_value, input_min, input_max, output_min, output_max):
    mapped_value = (input_value - input_min) * (output_max - output_min) // (input_max - input_min) + output_min
    mapped_value = max(min(mapped_value, output_max), output_min)
    return mapped_value

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
	# Get the number of buttons on the joystick
	num_buttons = joystick.get_numbuttons() 
	# Iterate through each button and print its index
for button_index in range(num_buttons):
	print("Button", button_index, ":", button_index)


while True:
	for event in pygame.event.get():
		if event.type == pygame.JOYBUTTONDOWN:  # Check for button press event
			button_id = event.button
			button_name = pygame.joystick.Joystick(0).get_button(button_id)
			print(f"Button {button_id} ({button_name}) pressed")

		#AXIS MOTION
		if event.type == pygame.JOYAXISMOTION:
			# Left Joystick X Axis
			if event.axis == 0:
				L3_x = event.value
				L3_x_result = map_input(L3_x, left_joystick_x_max, left_joystick_x_min, -127, 127)
				print(f"L3 X_AXIS Original Value: {L3_x}")
				# print(f"L3 X_AXIS Modified Value: {L3_x_result}")

			# Left Joystick Y Axis
			elif event.axis == 1:
				L3_y = event.value
				L3_y_result = map_input(L3_y, left_joystick_y_max, left_joystick_y_min, -127, 127)
				print(f"L3 Y_AXIS Original Value: {L3_y}")
				# print(f"L3 Y_AXIS Modified Value: {L3_y_result}")

			# Right Joystick X Axis
			if event.axis == 3:
				R3_x = event.value
				R3_x_result = map_input(R3_x, right_joystick_x_max, right_joystick_x_min, -127, 127)
				# print(f"R3 X_AXIS Original Value: {R3_x}")
				# print(f"R3 X_AXIS Modified Value: {R3_x_result}")

			# Right Joystick Y Axis
			elif event.axis == 4:
				R3_y = event.value
				R3_y_result = map_input(R3_y, right_joystick_y_max, right_joystick_y_min, -127, 127)
				# print(f"R3 Y_AXIS Original Value: {R3_y}")
				# print(f"R3 Y_AXIS Modified Value: {R3_y_result}")

		
		if event.type == pygame.JOYHATMOTION:
			if event.value == DPAD_UP:
				print("D-PAD UP")
			if event.value == DPAD_DOWN:
				print("DPAD_DOWN")
			if event.value == DPAD_LEFT:
				print("DPAD_LEFT")
			if event.value == DPAD_RIGHT:
				print("DPAD_RIGHT")

		if event.type == pygame.JOYBUTTONDOWN:			
			if event.button == BUTTON_SQUARE:
				print("BUTTON_SQUARE")
			if event.button == BUTTON_X:
				print("BUTTON_X")
			if event.button == BUTTON_CIRCLE:
				print("BUTTON_CIRCLE")
			if event.button == BUTTON_TRIANGLE:
				print("BUTTON_TRIANGLE")
			if event.button == BUTTON_SHARE:
				print("BUTTON_SHARE")
			if event.button == PS_BUTTON:
				print("PS_BUTTON")
			if event.button == OPTIONS_BUTTON:
				print("OPTIONS_BUTTON")
			if event.button == L2_IN:
				print("L_3_IN")
			if event.button == R2_IN:
				print("R_3_IN")
			if event.button == L_STICK_IN:
				print("L_STICK_IN")
			if event.button == R_STICK_IN:
				print("R_STICK_IN")
			if event.button == L1_IN:
				print("LEFT_BUMPER")
			if event.button == R2_IN:
				print("RIGHT_BUMPER")
			if event.button == TOUCH_PAD_CLICK_BUTTON:
				print("TOUCH_PAD_CLICK_BUTTON")
