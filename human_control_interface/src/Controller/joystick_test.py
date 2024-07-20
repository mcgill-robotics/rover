import pygame
import sys

def init_joystick():
    pygame.init()
    pygame.joystick.init()

    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("No joystick found.")
        return None
    else:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"Joystick initialized: {joystick.get_name()}")
        return joystick

def get_joystick_status(joystick):
    status = ""
    status += f"Buttons: {', '.join([str(joystick.get_button(i)) for i in range(joystick.get_numbuttons())])} | "
    status += f"Axes: {', '.join([str(joystick.get_axis(i)) for i in range(joystick.get_numaxes())])} | "
    status += f"Hats: {', '.join([str(joystick.get_hat(i)) for i in range(joystick.get_numhats())])}"
    return status

def main():
    joystick = init_joystick()
    if joystick is None:
        return

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        status = get_joystick_status(joystick)
        sys.stdout.write("\r" + status)
        sys.stdout.flush()

        pygame.time.wait(50)  # Wait for 1 second before updating again

    pygame.quit()

if __name__ == "__main__":
    main()
