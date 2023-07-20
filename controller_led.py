import pygame
import subprocess
import time

# Initialize pygame
pygame.init()
pygame.joystick.init()

# Function to change lightbar color using ds4drv command
def change_lightbar_color(r, g, b):
    color_hex = "#{:02X}{:02X}{:02X}".format(r, g, b)
    subprocess.run(["ds4drv", "set-lightbar", color_hex])

# Function to detect button events
def process_gamepad_events():
    for event in pygame.event.get():
        if event.type == pygame.JOYBUTTONDOWN:
            button_id = event.button
            if button_id == 0:  # Triangle button
                change_lightbar_color(255, 0, 0)  # Set lightbar color to red
            elif button_id == 1:  # Circle button
                change_lightbar_color(0, 255, 0)  # Set lightbar color to green
            elif button_id == 2:  # Cross (X) button
                change_lightbar_color(0, 0, 255)  # Set lightbar color to blue
            elif button_id == 3:  # Square button
                change_lightbar_color(255, 255, 0)  # Set lightbar color to yellow
        elif event.type == pygame.JOYBUTTONUP:
            # Reset lightbar color to white when button is released
            change_lightbar_color(255, 255, 255)

def main():
    try:
        # Connect to the first joystick (PS4 controller)
        gamepad = pygame.joystick.Joystick(0)
        gamepad.init()

        print("Press Triangle, Circle, X, or Square on your PS4 controller to change the lightbar color.")
        
        while True:
            process_gamepad_events()
            time.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        # Reset lightbar color to white before exiting
        change_lightbar_color(255, 255, 255)

if __name__ == "__main__":
    main()
