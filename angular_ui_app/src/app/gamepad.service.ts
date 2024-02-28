import { Injectable } from '@angular/core';

@Injectable({
  providedIn: 'root'
})
export class GamepadService {

  gamepad: Gamepad | null = null;
  gp_buttons: GamepadButton[] | null = null;

  axis_callback: (axis_v: number[]) => void;
  buttons_callback: (axis_h: number, i: number) => void;

  relay_messages: boolean = false;

  public enableGamepad(
      axis_callback: (axis_v: number[]) => void,
      buttons_callback: (axis_h: number, i: number) => void
    ) {
    window.addEventListener("gamepadconnected", (e: GamepadEvent) => {
      this.gamepad = navigator.getGamepads()[e.gamepad.index];
      this.gp_buttons = this.gamepad!.buttons.slice();
      this.axis_callback = axis_callback;
      this.buttons_callback = buttons_callback;
      setInterval(this.updateStatus.bind(this), 50);
    });

    window.addEventListener("gamepaddisconnected", () => {
      this.gamepad = null;
    });

    this.relay_messages = true;
  }

  public disableGamepad() {
    this.relay_messages = false;
  }

  private updateStatus() {
    if (this.gamepad && this.relay_messages) {
      let new_gp = navigator.getGamepads()[this.gamepad.index];

      this.axis_callback(new_gp!.axes.map((v) => v));

      // new_gp!.buttons.forEach((button, index) => {
      //   if (!button.pressed && this.gp_buttons![index].pressed) {
      //     this.buttons_callback(index, 0);
      //     // console.log(`Button ${index} released`);
      //   }
      // });
      this.gp_buttons = new_gp!.buttons.slice();
    }
    // sleep for 60th of a second
  }
}