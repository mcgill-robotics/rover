import { Component, HostListener, OnInit, OnDestroy } from '@angular/core';
import { GamepadService } from 'src/app/gamepad.service';
import { RosService } from 'src/app/ros.service';
import * as ROSLIB from 'roslib';

@Component({
  selector: 'app-drive-component',
  templateUrl: './drive.component.html',
  styleUrls: ['./drive.component.scss']
})

export class DriveComponent implements OnInit, OnDestroy {
  ros: ROSLIB.Ros;
  drive_twist_publisher: ROSLIB.Topic;
  pan_tilt_pub: ROSLIB.Topic;
  wheel_feedback_sub: ROSLIB.Topic;
  // wheel_velocity_cmd_sub: ROSLIB.Topic
  wheel_velocity_cmd_pub: ROSLIB.Topic

  pantilt_yaw: number = 0;
  pantilt_pitch: number = 90;
  angle_delta: number = 0.5;

  rover_linear_vel: number = 0.0;
  rover_angular_vel: number = 0.0;

  keyboard_sensitivity: number = 0.03;
  min_sensitivity: number = 0.01;
  max_sensitivity: number = 0.05;

  // max_linear_vel: number = 3;
  max_linear_vel: number = 50;
  max_angular_vel: number = 10;

  angular_velocity_scale: number = 0.25; // This will halve the angular velocity

  wheelFeedback = {
    leftFront: 0,
    leftBack: 0,
    rightFront: 0,
    rightBack: 0
  };

  wheelVelocityCmd = {
    leftFront: 0,
    leftBack: 0,
    rightFront: 0,
    rightBack: 0
  };


  keyStates: { [key: string]: boolean } = {};
  updateInterval: any;
  decayInterval: any;



  constructor(
    private gamepadService: GamepadService,
    private rosService: RosService
  ) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit() {
    this.initializeROSTopics();

    // this.setupKeyboardControl();

    // this.setupGamepadControl();
    this.gamepadService.connectControllerGamepad(this.controllerCallback.bind(this));

    console.log('Drive component initialized');
  }

  ngOnDestroy() {
    clearInterval(this.updateInterval);
    clearInterval(this.decayInterval);
    this.gamepadService.disableControllerGamepad();
    // if (this.wheel_velocity_cmd_sub) {
    //   this.wheel_velocity_cmd_sub.unsubscribe();
    // }
    if (this.wheel_feedback_sub) {
      this.wheel_feedback_sub.unsubscribe();
    }
  }

  initializeROSTopics() {
    this.drive_twist_publisher = new ROSLIB.Topic({
      ros: this.ros,
      name: '/rover_velocity_controller/cmd_vel',
      messageType: 'geometry_msgs/Twist'
    });

    this.pan_tilt_pub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/pantiltCmd',
      messageType: 'std_msgs/Float32MultiArray'
    });

    this.wheel_feedback_sub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/wheel_velocity_feedback',
      messageType: 'drive_control/WheelSpeed'
    });

    this.wheel_feedback_sub.subscribe((message: any) => {
      this.wheelFeedback.leftFront = message.left[1];
      this.wheelFeedback.leftBack = message.left[0];
      this.wheelFeedback.rightFront = message.right[1];
      this.wheelFeedback.rightBack = message.right[0];
    });

    // this.wheel_velocity_cmd_sub = new ROSLIB.Topic({
    //   ros: this.ros,
    //   name: '/wheel_velocity_cmd',
    //   messageType: 'drive_control/WheelSpeed',
    // });

    // this.wheel_velocity_cmd_sub.subscribe((message: any) => {
    //   this.wheelVelocityCmd.leftFront = message.left[1];
    //   this.wheelVelocityCmd.leftBack = message.left[0];
    //   this.wheelVelocityCmd.rightFront = message.right[1];
    //   this.wheelVelocityCmd.rightBack = message.right[0];
    // });

    this.wheel_velocity_cmd_pub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/wheel_velocity_cmd',
      messageType: 'drive_control/WheelSpeed',
    });

  }

  private filterVelocity(value: number, threshold: number): number {
    return Math.abs(value) < threshold ? 0 : value;
  }

  private controllerCallback(input: { [key: string]: number | boolean }) {

    // Update rover velocities based on gamepad input
    // this.rover_linear_vel = -input_dir['axis1'] as number * this.max_linear_vel;
    // this.rover_angular_vel = -input_dir['axis3'] as number * this.max_angular_vel;
    // this.publishDriveTwist();

    const velocityThreshold = 3;  // Define the velocity threshold

    this.wheelVelocityCmd.leftFront = -input['axis1'] as number * this.max_linear_vel;
    this.wheelVelocityCmd.leftBack = -input['axis1'] as number * this.max_linear_vel;
    this.wheelVelocityCmd.rightFront = -input['axis3'] as number * this.max_linear_vel;
    this.wheelVelocityCmd.rightBack = -input['axis3'] as number * this.max_linear_vel;

    this.wheelVelocityCmd.leftFront = this.filterVelocity(this.wheelVelocityCmd.leftFront, velocityThreshold);
    this.wheelVelocityCmd.leftBack = this.filterVelocity(this.wheelVelocityCmd.leftBack, velocityThreshold);
    this.wheelVelocityCmd.rightFront = this.filterVelocity(this.wheelVelocityCmd.rightFront, velocityThreshold);
    this.wheelVelocityCmd.rightBack = this.filterVelocity(this.wheelVelocityCmd.rightBack, velocityThreshold);

    // console.log("Wheel speed", this.wheelVelocityCmd);
    this.publishWheelSpeed();

    if (input['up']) {
      // this.pantilt_pitch = this.clamp(this.pantilt_pitch + this.angle_delta);
    }
    if (input['down']) {
      // this.pantilt_pitch = this.clamp(this.pantilt_pitch - this.angle_delta);
    }
    if (input['left']) {
      this.pantilt_yaw = this.clamp(this.pantilt_yaw + this.angle_delta);
    }
    if (input['right']) {
      this.pantilt_yaw = this.clamp(this.pantilt_yaw - this.angle_delta);
    }

    this.pan_tilt_pub.publish({ data: [this.pantilt_pitch, this.pantilt_yaw] });
  }
  setupGamepadControl() {

  }

  setupKeyboardControl() {
    this.updateInterval = setInterval(() => this.updateMovement(), 10);
    this.decayInterval = setInterval(() => this.decayVelocity(), 10);
  }

  @HostListener('window:keydown', ['$event'])
  keyDownEvent(event: KeyboardEvent) {
    this.keyStates[event.key.toLowerCase()] = true;
  }

  @HostListener('window:keyup', ['$event'])
  keyUpEvent(event: KeyboardEvent) {
    if (event.repeat) return;
    this.keyStates[event.key.toLowerCase()] = false;
  }

  updateMovement() {
    let keyboard_accumulator_linear = 0.0;
    let keyboard_accumulator_twist = 0.0;

    if (this.keyStates['w']) keyboard_accumulator_linear += this.keyboard_sensitivity;
    if (this.keyStates['s']) keyboard_accumulator_linear -= this.keyboard_sensitivity;
    if (this.keyStates['a']) keyboard_accumulator_twist -= this.keyboard_sensitivity;
    if (this.keyStates['d']) keyboard_accumulator_twist += this.keyboard_sensitivity;
    if (this.keyStates[' ']) {
      this.rover_linear_vel = 0.0;
      this.rover_angular_vel = 0.0;
    }

    keyboard_accumulator_linear = Math.max(Math.min(keyboard_accumulator_linear, 1.0), -1.0);
    keyboard_accumulator_twist = Math.max(Math.min(keyboard_accumulator_twist, 1.0), -1.0);
    this.rover_linear_vel += this.max_linear_vel * keyboard_accumulator_linear;
    this.rover_angular_vel += this.max_angular_vel * keyboard_accumulator_twist * this.angular_velocity_scale;

    this.publishDriveTwist();
  }

  decayVelocity() {
    const is_accelerating_linear = this.keyStates['w'] || this.keyStates['s'];
    const is_accelerating_twist = this.keyStates['a'] || this.keyStates['d'];

    if (is_accelerating_linear) {
      this.rover_linear_vel *= 0.96;
    } else {
      this.rover_linear_vel *= 0.9;
    }

    if (is_accelerating_twist) {
      this.rover_angular_vel *= 0.99;
    } else {
      this.rover_angular_vel *= 0.9;
    }

    this.publishDriveTwist();
  }

  publishDriveTwist() {
    // console.log(`Publishing twist - Linear: ${this.rover_linear_vel}, Angular: ${this.rover_angular_vel}`);

    const roverTwist = new ROSLIB.Message({
      linear: { x: this.rover_linear_vel, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: this.rover_angular_vel }
    });

    this.drive_twist_publisher.publish(roverTwist);
  }

  publishWheelSpeed() {
    const wheelSpeedMessage = new ROSLIB.Message({
      left: [this.wheelVelocityCmd.leftBack, this.wheelVelocityCmd.leftFront],
      right: [this.wheelVelocityCmd.rightBack, this.wheelVelocityCmd.rightFront]
    });

    this.wheel_velocity_cmd_pub.publish(wheelSpeedMessage);
  }

  enableGamepad() {
    this.gamepadService.enableControllerGamepad();
  }

  disableGamepad() {
    this.gamepadService.disableControllerGamepad();
  }

  private clamp(new_angle: number): number {
    return Math.max(0, Math.min(new_angle, 180));
  }

  updateSensitivity(event: Event): void {
    const inputElement = event.target as HTMLInputElement;
    if (inputElement && inputElement.value) {
      this.keyboard_sensitivity = parseFloat(inputElement.value);
    }
  }

  resetSlider(): void {
    this.keyboard_sensitivity = 0.03;  // Reset to the initial or desired default value
  }
}