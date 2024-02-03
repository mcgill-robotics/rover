import { Component } from '@angular/core';
import * as ROSLIB from 'roslib';
import { RosService } from 'src/app/ros.service';

@Component({
  selector: 'app-drive-component',
  templateUrl: './drive.component.html',
  styleUrls: ['./drive.component.scss']
})
export class DriveComponent {
  // ROS fields
  ros: ROSLIB.Ros;
  wheels_sub: ROSLIB.Topic; //subscriber
  position_sub: ROSLIB.Topic

  // Page fields
  wheels : {
    right_front: Number;
    left_front: Number;
    right_back: Number;
    left_back: Number;
  }

  msgstr: String;

  // orientation : {
  //   w: Number;
  //   x: Number;
  //   y: Number;
  //   z: Number;
  // }

  // position : {
  //   x: Number;
  //   y: Number;
  //   z: Number;
  // }

  // latitude: Number = 0;
  // longitude: Number = 0;
  wheelSpeed: ROSLIB.Message; //payload to be sent
  payload: {
    orientation : {
      w: Number;
      x: Number;
      y: Number;
      z: Number;
    },
    position : {
      x: Number;
      y: Number;
      z: Number;
    }
  }

  constructor(private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit() {

  
    this.position_sub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/position_pose', //may be odrive
      messageType: 'geometry_msgs/Pose'
    });
    
    this.wheels_sub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/driveFb"', //may be odrive
      messageType: 'std_msgs/Float32MultiArray'
    });

    // this.antennaInstruction = new ROSLIB.Message({
    //   data : [0, 0]
    // });

    // starts the liistener
    this.wheel_listen();
    this.position_listen();
  }

  // publish(payload: string) {
  //   this.pub.publish({data: payload});
  // }

  
  wheel_listen() {
    this.wheels_sub.subscribe((message: any) => {
      console.log('Received message on ' + this.wheels_sub.name + ': ' + message.data);
    });

  }

  position_listen() {
    this.position_sub.subscribe((message: any) => {
      this.payload = message;
      this.msgstr = JSON.stringify(message);
      
    });

  }
}

