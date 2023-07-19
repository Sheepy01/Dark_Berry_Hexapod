
import pygame

#CONSTANTS

PS4_BUTTON_SQUARE = 0
# Initialize variables for maximum and minimum values
left_joystick_x_max = -1.0
left_joystick_x_min = 1.0
left_joystick_y_max = -1.0
left_joystick_y_min = 1.0

right_joystick_x_max = -1.0
right_joystick_x_min = 1.0
right_joystick_y_max = -1.0
right_joystick_y_min = 1.0

BUTTON_X = 0
BUTTON_CIRCLE = 1
BUTTON_SQUARE = 2
BUTTON_TRIANGLE = 3

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

		if event.type == pygame.JOYAXISMOTION:

			# Left Joystick X Axis
			if event.axis == 0:
				L3_x = event.value
				L3_x_result = map_input(L3_x, left_joystick_x_max, left_joystick_x_min, -127, 127)
				print(f"L3 X_AXIS Original Value: {L3_x}")
				print(f"L3 X_AXIS Modified Value: {L3_x_result}")

			# Left Joystick Y Axis
			elif event.axis == 1:
				L3_y = event.value
				L3_y_result = map_input(L3_y, left_joystick_y_max, left_joystick_y_min, -127, 127)
				print(f"L3 Y_AXIS Original Value: {L3_y}")
				print(f"L3 Y_AXIS Modified Value: {L3_y_result}")

			# Right Joystick X Axis
			if event.axis == 2:
				R3_x = event.value
				R3_x_result = map_input(R3_x, right_joystick_x_max, right_joystick_x_min, -127, 127)
				print(f"R3 X_AXIS Original Value: {R3_x}")
				print(f"R3 X_AXIS Modified Value: {R3_x_result}")

			# Right Joystick Y Axis
			elif event.axis == 3:
				R3_y = event.value
				R3_y_result = map_input(R3_y, right_joystick_y_max, right_joystick_y_min, -127, 127)
				print(f"R3 Y_AXIS Original Value: {R3_y}")
				print(f"R3 Y_AXIS Modified Value: {R3_y_result}")

		if event.type == pygame.JOYBUTTONDOWN:
			
			if event.button == pygame.CONTROLLER_BUTTON_DPAD_UP:
				pass
			if event.button == pygame.CONTROLLER_BUTTON_DPAD_DOWN:
				pass
			if event.button == pygame.CONTROLLER_BUTTON_DPAD_LEFT:
				pass
			if event.button == pygame.CONTROLLER_BUTTON_DPAD_RIGHT:
				pass

			if event.button == BUTTON_SQUARE:
				print("Square")
			if event.button == BUTTON_X:
				print("X")
			if event.button == BUTTON_CIRCLE:
				print("Circle")
			if event.button == BUTTON_TRIANGLE:
				print("tri")