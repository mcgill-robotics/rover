import { Injectable } from '@angular/core';

@Injectable({
  providedIn: 'root'
})
export class GamepadService {

  gamepad: Gamepad | null = null;
  gp_buttons: GamepadButton[] | null = null;

  constructor() {

  }

  public enable_gamepad() {
    window.addEventListener("gamepadconnected", (e: GamepadEvent) => {
      this.gamepad = navigator.getGamepads()[e.gamepad.index];
      this.gp_buttons = this.gamepad!.buttons.slice();
      setInterval(this.updateStatus.bind(this), 50);
    });

    window.addEventListener("gamepaddisconnected", () => {
      this.gamepad = null;
    });
  }

  private updateStatus() {
    if (this.gamepad) {
      let new_gp = navigator.getGamepads()[this.gamepad.index];

      new_gp!.axes.forEach((axis, index) => {
        console.log(`Axis ${index} moved to ${axis}`);
      });

      new_gp!.buttons.forEach((button, index) => {
        if (!button.pressed && this.gp_buttons![index].pressed) {
          console.log(`Button ${index} released`);
        }
      });
      this.gp_buttons = new_gp!.buttons.slice();
    }
    // sleep for 60th of a second
  }
}
