import { Component } from '@angular/core';
import * as ROSLIB from 'roslib';
import { RosService } from 'src/app/ros.service';
import { GamepadService } from '../../gamepad.service';

@Component({
  selector: 'app-o-drive',
  templateUrl: './o-drive.component.html',
  styleUrls: ['./o-drive.component.scss']
})
export class ODriveComponent {
  // ROS fields
  ros: ROSLIB.Ros;
  wheels_sub: ROSLIB.Topic;
  position_sub: ROSLIB.Topic;

  wheels: {
    right: Number[];
    left: Number[];
  }

  constructor(private rosService: RosService, private gamepadService: GamepadService) {
    this.ros = this.rosService.getRos();
  }

  // starting hook
  ngOnInit() {
    this.gamepadService.enable_gamepad();

    this.wheels_sub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/wheel_velocity_cmd', //may switch to odrive
      messageType: 'drive_control/WheelSpeed'
    });

    this.wheel_listen();
  }

  // functions
  wheel_listen() {
    this.wheels_sub.subscribe((message: any) => {
      console.log('Received message on ' + this.wheels_sub.name + ': ' + JSON.stringify(message));
      this.wheels = message;

    });

  }

}
