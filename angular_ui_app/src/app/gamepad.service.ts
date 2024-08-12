import { Injectable } from '@angular/core';

@Injectable({
  providedIn: 'root'
})
export class GamepadService {

  controller_gamepad: Gamepad | null = null;
  joystick_gamepad: Gamepad | null = null;

  controller_buttons_state: GamepadButton[] | null = null;
  joystick_input_state: { [key: string]: number | boolean } | null = null;

  controller_callback: (axis_v: { [key: string]: number | boolean }) => void;
  joystick_callback: (input: { [key: string]: number | boolean }) => void;

  buttons_callback_controller: (axis_h: number, i: number) => void;
  buttons_callback_joystick: (axis_h: number, i: number) => void;

  relay_messages_controller: boolean = false;
  relay_messages_controller_string: string = 'Off';
  relay_messages_joystick: boolean = false;
  relay_messages_joystick_string: string = 'Off';

  public connectControllerGamepad(
    controller_callback: (input_dir: { [key: string]: number | boolean }) => void,
  ) {
    window.addEventListener("gamepadconnected", (e: GamepadEvent) => {
      console.log("Gamepad connected at index=%d: e.gamepad!.index=%s", e.gamepad!.index, e.gamepad!.id);
      if (!(e.gamepad!.id.startsWith("Logitech"))) {
        // if (e.gamepad!.id.startsWith("Sony")) {
        this.controller_gamepad = e.gamepad;
        this.controller_callback = controller_callback;
        console.log("Controller connected");
        setInterval(this.updateControllerStatus.bind(this), 10);
      }
    });

    window.addEventListener("gamepaddisconnected", (e: GamepadEvent) => {
      if (e.gamepad!.id.startsWith("Sony")) {
        this.controller_gamepad = null;
      }
    });
  }

  public connectJoystickGamepad(
    joystick_callback: (input: { [key: string]: number | boolean }) => void,
  ) {
    window.addEventListener("gamepadconnected", (e: GamepadEvent) => {
      if (e.gamepad!.id.startsWith("Logitech")) {
        this.joystick_gamepad = e.gamepad;
        this.joystick_input_state = this.getJoystickInputDir(this.joystick_gamepad!);
        this.joystick_callback = joystick_callback;
        console.log("Joystick connected");
        setInterval(this.updateJoystickStatus.bind(this), 10);
      }
    });

    window.addEventListener("gamepaddisconnected", (e: GamepadEvent) => {
      if (e.gamepad!.id.startsWith("Logitech")) {
        this.joystick_gamepad = null;
      }
    });
  }

  public enableControllerGamepad() {
    this.relay_messages_controller = true;
    this.relay_messages_controller_string = 'On';
  }

  public enableJoystickGamepad() {
    this.relay_messages_joystick = true;
    this.relay_messages_joystick_string = 'On';
  }

  public disableControllerGamepad() {
    this.relay_messages_controller = false;
    this.relay_messages_controller_string = 'Off';
  }

  public disableJoystickGamepad() {
    this.relay_messages_joystick = false;
    this.relay_messages_joystick_string = 'Off';
  }

  private updateControllerStatus() {
    console.log("this.controller_gamepad=%s, this.relay_messages_controller=%s", this.controller_gamepad, this.relay_messages_controller);
    if (this.controller_gamepad && this.relay_messages_controller) {
      let new_gp = navigator.getGamepads()[this.controller_gamepad.index];
      // this.controller_callback(new_gp!.axes.map((v) => v));

      // new_gp!.buttons.forEach((button, index) => {
      //   if (!button.pressed && this.controller_buttons_state![index].pressed) {
      //     this.buttons_callback_controller(index, 0);
      //     console.log(`Button ${index} released`);
      //   }
      // });

      this.controller_callback(this.getControllerInputDir(new_gp!));
    }
    // sleep for 60th of a second
  }

  private updateJoystickStatus() {
    if (this.joystick_gamepad && this.relay_messages_joystick) {
      let new_gp = navigator.getGamepads()[this.joystick_gamepad.index];
      // console.log(new_gp!.axes);
      this.joystick_callback(this.getJoystickInputDir(new_gp!));
    }
  }

  private getJoystickInputDir(gmpd: Gamepad) {
    console.log("Joystick axes: ", gmpd.axes);
    return {
      "a1": gmpd.axes[0],
      "a2": gmpd.axes[1],
      "a3": gmpd.axes[2],
      "a4": gmpd.axes[3],
      "a5": gmpd.axes[4],
      "a6": gmpd.axes[5],
      "a7": gmpd.axes[6],
      "b1": gmpd.buttons[0].pressed,
      "b2": gmpd.buttons[1].pressed,
      "b3": gmpd.buttons[2].pressed,
      "b4": gmpd.buttons[3].pressed,
      "b5": gmpd.buttons[4].pressed,
      "b6": gmpd.buttons[5].pressed,
      "b7": gmpd.buttons[6].pressed,
      "b8": gmpd.buttons[7].pressed,
      "b9": gmpd.buttons[8].pressed,
      "b10": gmpd.buttons[9].pressed,
      "b11": gmpd.buttons[10].pressed,
      "b12": gmpd.buttons[11].pressed
    };
  }

  private getControllerInputDir(gmpd: Gamepad) {
    console.log("Controller axes getControllerInputDir: ", gmpd.axes);
    return {
      "axis0": gmpd.axes[0],
      "axis1": gmpd.axes[1],
      "axis2": gmpd.axes[2],
      "axis3": gmpd.axes[3],

      "a2": -gmpd.axes[1],
      "a4": gmpd.axes[2],
      "b0": gmpd.buttons[0].pressed,
      "b1": gmpd.buttons[1].pressed,
      "b2": gmpd.buttons[2].pressed,
      "b3": gmpd.buttons[3].pressed,
      "b4": gmpd.buttons[4].pressed,
      "b5": gmpd.buttons[5].pressed,
      "b6": gmpd.buttons[6].pressed,
      "b7": gmpd.buttons[7].pressed,
      "b8": gmpd.buttons[8].pressed,
      "b9": gmpd.buttons[9].pressed,
      "b10": gmpd.buttons[10].pressed,
      "b11": gmpd.buttons[11].pressed,

      "left": gmpd.buttons[14].pressed,
      "right": gmpd.buttons[15].pressed,
      "up": gmpd.buttons[12].pressed,
      "down": gmpd.buttons[13].pressed
    }
  }
}
