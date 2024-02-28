import { Component } from '@angular/core';
import * as ROSLIB from 'roslib';
import { RosService } from 'src/app/ros.service';

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

  // feedback data
  wheels: {
    right: Number[];
    left: Number[];
  }

  constructor(private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit() {
    this.wheels_sub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/wheel_velocity_cmd', //using sent odrive cmd, should we use odrive feedback instead, not tested yet
      messageType: 'drive_control/WheelSpeed' //to be implemented
    });

    this.wheel_listen(); 
  }

  // functions
  wheel_listen() {
    this.wheels_sub.subscribe((message: any) => {
      this.wheels = message;
    });

  }

}
