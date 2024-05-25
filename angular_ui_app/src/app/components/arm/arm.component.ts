<<<<<<< HEAD
import { Component, Input} from '@angular/core';
import * as ROSLIB from 'roslib';
import { RosService } from 'src/app/ros.service';

=======
import { Component, Input, Output, EventEmitter} from '@angular/core';
import * as ROSLIB from 'roslib';
import { GamepadService } from 'src/app/gamepad.service';
import { RosService } from 'src/app/ros.service';
>>>>>>> 6f7ca508434d866472d199e57b0bb4593daac748

@Component({
  selector: 'app-arm-component',
  templateUrl: './arm.component.html',
  styleUrls: ['./arm.component.scss']
})
export class ArmComponent {
  ros: ROSLIB.Ros;     
  arm_sub_brushed: ROSLIB.Topic;
  arm_sub_brushless: ROSLIB.Topic;

  @Input() initialValue: number = 0;

  // array contains initial values 
  array  : number[]= [0,0,0];
  prev_joystick_input_state: { [key: string]: number | boolean } | null = null;
  last_arm_control_mode: number = 1;
  maxVelPercentage: number = 0;

  ros: ROSLIB.Ros;
  gamepad_arm_control_pub: ROSLIB.Topic;
  arm_brushed_sub: ROSLIB.Topic;
  arm_brushless_sub: ROSLIB.Topic;

  arm_brushed_FB = [0,0,0];
  arm_brushless_FB = [0,0,0];

  constructor(private gamepadService: GamepadService, private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  armBrushed: {
    wrist: number[];
    elbow: Number[];
    shoulder: Number[];
  } = {
    wrist: [0],
    elbow: [0],
    shoulder: [0]
  };

  armBrushless: {
    wrist: number[];
    elbow: Number[];
    shoulder: Number[];
  } = {
    wrist: [0],
    elbow: [0],
    shoulder: [0]
  };

  increment(id: number) {
    console.log("id: ", id);
    this.array[id] += 0.5;
    console.log("val 0: ", this.array[id]);
  }

  decrement(id: number) {
    this.array[id] -= 0.5;
  }

<<<<<<< HEAD
  constructor(private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit() {
    this.arm_sub_brushed = new ROSLIB.Topic({
=======
  ngOnInit() {
    this.gamepad_arm_control_pub = new ROSLIB.Topic({
      ros : this.ros,
      name : '/arm_controller_input',
      messageType : 'arm_control/ArmControllerInput'
    });

    this.arm_brushed_sub = new ROSLIB.Topic({
>>>>>>> 6f7ca508434d866472d199e57b0bb4593daac748
      ros: this.ros,
      name: '/armBrushedFB',
      messageType: 'std_msgs/Float32MultiArray'
    });

<<<<<<< HEAD
    this.arm_sub_brushless = new ROSLIB.Topic({
=======
    this.arm_brushless_sub = new ROSLIB.Topic({
>>>>>>> 6f7ca508434d866472d199e57b0bb4593daac748
      ros: this.ros,
      name: '/armBrushlessFB',
      messageType: 'std_msgs/Float32MultiArray'
    });

<<<<<<< HEAD
    this.arm_listen();

  }
  arm_listen() {
    this.arm_sub_brushed.subscribe((message: any) => {
      this.armBrushed.wrist = message.data[0].toFixed(3);
      this.armBrushed.elbow = message.data[1].toFixed(3);
      this.armBrushed.shoulder = message.data[2].toFixed(3);
    })

    this.arm_sub_brushless.subscribe((message: any) => {
      this.armBrushless.wrist = message.data[0].toFixed(3);
      this.armBrushless.elbow = message.data[1].toFixed(3);
      this.armBrushless.shoulder = message.data[2].toFixed(3);
    })
=======
    this.arm_brushed_sub.subscribe((message: any) => {
      this.arm_brushed_FB = message.data;
    });

    this.arm_brushless_sub.subscribe((message: any) => {
      this.arm_brushless_FB = message.data;
    });

    this.gamepadService.connectJoystickGamepad(
      (input_dir: { [key: string]: number | boolean }) => {
        if (this.prev_joystick_input_state == null) {
          this.prev_joystick_input_state = input_dir;
        }

        this.maxVelPercentage = (1 - ((input_dir['a4'] as number) + 1) / 2) / 8;

        let msg = {
          X_dir: (input_dir['a2'] as number) ** 2,
          Y_dir: (input_dir['a1'] as number) ** 2,
          Z_dir: (input_dir['a3'] as number) ** 2,
          MaxVelPercentage: this.maxVelPercentage,
          Mode: this.last_arm_control_mode,
        }

        if (input_dir['a2'] as number > 0) {
          msg.X_dir = -1 * msg.X_dir;
        }

        if (input_dir['a1'] as number > 0) {
          msg.Y_dir = -1 * msg.Y_dir;
        }

        if (input_dir['a3'] as number < 0) {
          msg.Z_dir = -1 * msg.Z_dir;
        }

        if (this.risingEdge(this.prev_joystick_input_state['b8'] as boolean, input_dir['b8'] as boolean)) {
          msg.Mode = 1;
        } else if (this.risingEdge(this.prev_joystick_input_state['b10'] as boolean, input_dir['b10'] as boolean)) {
          msg.Mode = 2;
        } else if (this.risingEdge(this.prev_joystick_input_state['b12'] as boolean, input_dir['b12'] as boolean)) {
          msg.Mode = 3;
        } else if (this.risingEdge(this.prev_joystick_input_state['b7'] as boolean, input_dir['b7'] as boolean)) {
          msg.Mode = 4;
        } else if (this.risingEdge(this.prev_joystick_input_state['b9'] as boolean, input_dir['b9'] as boolean)) {
          msg.Mode = 5;
        } else if (this.risingEdge(this.prev_joystick_input_state['b11'] as boolean, input_dir['b11'] as boolean)) {
          msg.Mode = 0;
        }

        this.gamepad_arm_control_pub.publish(msg)
        this.prev_joystick_input_state = input_dir;
        this.last_arm_control_mode = msg.Mode;
      },
    );
  }

  private risingEdge(prev: boolean, current: boolean): boolean {
    return current && !prev;
  }

  enableGamepad() {
    this.gamepadService.enableJoystickGamepad();
  }

  disableGamepad() {
    this.gamepadService.disableJoystickGamepad();
  }

  formatNumber(value: number) {
    return value.toFixed(3);
>>>>>>> 6f7ca508434d866472d199e57b0bb4593daac748
  }
}

