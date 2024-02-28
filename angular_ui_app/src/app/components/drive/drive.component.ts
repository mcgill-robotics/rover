import { Component } from '@angular/core';
import { GamepadService } from '../../gamepad.service';
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
  gagamepadService: GamepadService;

  constructor(private gamepadService: GamepadService, private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit() {
    this.gamepad_drive_pub = new ROSLIB.Topic({
      ros : this.ros,
      name : '/angular_ui_app/drive',
      messageType : 'std_msgs/Float32MultiArray'
    });
  }

  enableGamepad() {
    this.gamepadService.enableGamepad(
      (axis_v) => {
        this.gamepad_drive_pub.publish({data: [-axis_v[1], axis_v[2]]});
      },
      (button_v, i) => {}
    );
  }

  disableGamepad() {
    this.gamepadService.disableGamepad();
  }
}