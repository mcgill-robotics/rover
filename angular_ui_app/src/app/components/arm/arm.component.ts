import { Component, Input, OnInit } from '@angular/core';
import * as ROSLIB from 'roslib';
import { GamepadService } from 'src/app/gamepad.service';
import { RosService } from 'src/app/ros.service';

interface Joint {
  name: string;
  button: number;
  direction: number;
  axis?: number;
  position: number;
}

@Component({
  selector: 'app-arm-component',
  templateUrl: './arm.component.html',
  styleUrls: ['./arm.component.scss']
})
export class ArmComponent implements OnInit {
  @Input() initialValue: number = 0;

  // Array to hold joint configurations
  joints: Joint[] = [
    { name: "joint_elbow", button: 5, direction: -1, position: 0.0 },
    { name: "joint_shoulder", button: 1, direction: -1, position: 0.0 },
    { name: "joint_waist", button: 2, direction: -1, axis: 6, position: 0.0 },
    { name: "joint_end_effector", button: 4, direction: -1, position: 0.0 },
    { name: "joint_wrist_roll", button: 6, direction: 1, axis: 1, position: 0.0 },
    { name: "joint_wrist_pitch", button: 3, direction: 1, position: 0.0 },
    { name: "joint_7", button: 7, direction: 1, position: 0.0 },
  ];

  baseAngleIncrement: number = 1e-2; // Base increment step size
  minSpeedMultiplier: number = 0.1;
  maxSpeedMultiplier: number = 10;
  angleIncrement: number = this.baseAngleIncrement;
  axisThreshold: number = 0.2; // Threshold for joystick axis movement

  ros: ROSLIB.Ros;
  brushed_pub: ROSLIB.Topic;
  brushless_pub: ROSLIB.Topic;
  arm_brushed_sub: ROSLIB.Topic;
  arm_brushless_sub: ROSLIB.Topic;
  arm_brushed_FB = [0, 0, 0];
  arm_brushless_FB = [0, 0, 0];

  constructor(private gamepadService: GamepadService, private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit() {
    this.brushed_pub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/armBrushedCmd',
      messageType: 'std_msgs/Float32MultiArray'
    });

    this.brushless_pub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/armBrushlessCmd',
      messageType: 'std_msgs/Float32MultiArray'
    });


    this.arm_brushed_sub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/armBrushedFB',
      messageType: 'std_msgs/Float32MultiArray'
    });

    this.arm_brushless_sub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/armBrushlessFB',
      messageType: 'std_msgs/Float32MultiArray'
    });

    this.arm_brushed_sub.subscribe((message: any) => {
      this.arm_brushed_FB = message.data;
    });

    this.arm_brushless_sub.subscribe((message: any) => {
      this.arm_brushless_FB = message.data;
    });

    // Setup gamepad connection
    this.gamepadService.connectControllerGamepad(this.controllerCallback.bind(this));
    this.gamepadService.connectJoystickGamepad(this.joystickCallback.bind(this));
  }

  private mapRange(value: number, inMin: number, inMax: number, outMin: number, outMax: number): number {
    return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
  }

  private getJointByName(name: string): Joint | null {
    return this.joints.find(joint => joint.name === name) || null;
  }

  private publishJointStates() {
    const brushedMsg = new ROSLIB.Message({
      data: [
        this.getJointByName("joint_end_effector")?.position ?? 0,
        this.getJointByName("joint_wrist_roll")?.position ?? 0,
        this.getJointByName("joint_wrist_pitch")?.position ?? 0
      ]
    });

    const brushlessMsg = new ROSLIB.Message({
      data: [
        this.getJointByName("joint_elbow")?.position ?? 0,
        this.getJointByName("joint_shoulder")?.position ?? 0,
        this.getJointByName("joint_waist")?.position ?? 0
      ]
    });

    this.brushed_pub.publish(brushedMsg);
    this.brushless_pub.publish(brushlessMsg);

    // Log joint states
    const jointStates = this.joints.map(joint => `${joint.name}: ${joint.position.toFixed(2)} degrees`).join(' | ');
    console.log(jointStates);
  }

  private controllerCallback(input: { [key: string]: number | boolean }) {
    this.updateJoints(input);
  }

  private joystickCallback(input: { [key: string]: number | boolean }) {
    this.updateJoints(input);
  }

  private updateJoints(input: { [key: string]: number | boolean }) {
    const speedControlValue = input['a7'] as number; // Speed control on axis 4
    const speedMultiplier = this.mapRange(speedControlValue, -1, 1, this.maxSpeedMultiplier, this.minSpeedMultiplier);
    this.angleIncrement = this.baseAngleIncrement * speedMultiplier;

    this.joints.forEach(joint => {


      const key = `a${joint.axis || 2}`;
      const axisValue = input[key] as number; // Default to axis 1 if not specified
      console.log(`key=${key} axisValue=${axisValue}`);
      if (Math.abs(axisValue) > this.axisThreshold) {
        if (input[`b${joint.button}`]) {
          joint.position += this.angleIncrement * axisValue * joint.direction;
        }
      }
    });

    // Check if reset buttons are pressed
    if (input['b10']) { // Button 10 for brushless joints reset
      this.getJointByName("joint_elbow")!.position = 0.0;
      this.getJointByName("joint_shoulder")!.position = 0.0;
      this.getJointByName("joint_waist")!.position = 0.0;
    }

    if (input['b11']) { // Button 11 for brushed joints reset
      this.getJointByName("joint_end_effector")!.position = 0.0;
      this.getJointByName("joint_wrist_roll")!.position = 0.0;
      this.getJointByName("joint_wrist_pitch")!.position = 0.0;
    }

    this.publishJointStates();
  }

  enableGamepad() {
    this.gamepadService.enableJoystickGamepad();
  }

  disableGamepad() {
    this.gamepadService.disableJoystickGamepad();
  }
}
