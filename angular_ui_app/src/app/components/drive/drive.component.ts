import { Component } from '@angular/core';
import { GamepadService } from 'src/app/gamepad.service';
import { RosService } from 'src/app/ros.service';
import * as ROSLIB from 'roslib';

@Component({
  selector: 'app-drive-component',
  templateUrl: './drive.component.html',
  styleUrls: ['./drive.component.scss']
})
export class DriveComponent {
  ros: ROSLIB.Ros;
  gamepad_drive_pub: ROSLIB.Topic;
  pan_tilt_pub: ROSLIB.Topic;

  gagamepadService: GamepadService;

  pantilt_yaw: number = 0;
  pantilt_pitch: number = 90;

  angle_delta: number = 0.5;

  constructor(private gamepadService: GamepadService, private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit() {
    this.gamepad_drive_pub = new ROSLIB.Topic({
      ros : this.ros,
      name : '/angular_ui_app/drive',
      messageType : 'std_msgs/Float32MultiArray'
    });

    this.pan_tilt_pub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/pantiltCmd',
      messageType: 'std_msgs/Float32MultiArray'
    })

    this.gamepadService.connectControllerGamepad(
      (input_dir: { [key: string]: number | boolean }) => {
        this.gamepad_drive_pub.publish({data: [input_dir['a2'], input_dir['a4']]});

        if (input_dir['up']) {
          this.pantilt_pitch = this.clamp(this.pantilt_pitch + this.angle_delta);
        }
        if (input_dir['down']) {
          this.pantilt_pitch = this.clamp(this.pantilt_pitch - this.angle_delta);
        }
        if (input_dir['left']) {
          this.pantilt_yaw = this.clamp(this.pantilt_yaw + this.angle_delta);
        }
        if (input_dir['right']) {
          this.pantilt_yaw = this.clamp(this.pantilt_yaw - this.angle_delta);;
        }

        this.pan_tilt_pub.publish({data: [this.pantilt_pitch, this.pantilt_yaw]})
      },
    );
  }

  enableGamepad() {
    this.gamepadService.enableControllerGamepad();
  }

  disableGamepad() {
    this.gamepadService.disableControllerGamepad();
  }

  private clamp(new_angle: number) {
    return Math.max(0, Math.min(new_angle, 180))
  }
}