
import sys
import time
from colorama import Fore, Style

def loading_animation():
    animation_chars = "⣾⣽⣻⢿⡿⣟⣯⣷"
    for i in range(30):  # Display 30 frames
        progress = (i + 1) / 30
        bar_width = 30
        filled_width = int(bar_width * progress)
        bar = animation_chars[0] * filled_width + "-" * (bar_width - filled_width)
        sys.stdout.write("\r" + Fore.GREEN + "Loading: [" + bar + "]" + Fore.CYAN + " {:.0%}".format(progress))
        sys.stdout.flush()
        time.sleep(0.1)  # Adjust the delay to control animation speed

def loading_screen():
    print(Fore.YELLOW + "Initializing...")
    time.sleep(2)  # Delay for 2 seconds
    loading_animation()
    print("\n" + Style.RESET_ALL + "Done!")