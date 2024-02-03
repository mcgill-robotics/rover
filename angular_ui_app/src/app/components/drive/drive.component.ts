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
  wheels_sub: ROSLIB.Topic;
  position_sub: ROSLIB.Topic

  // to be deleted after test output
  msgstr: String;
  cmpstr: String;

  // wheel fields are ignored for the moment since no corresponding publisher/topic to subscribe to
  // wheels : {
  //   right_front: Number;
  //   left_front: Number;
  //   right_back: Number;
  //   left_back: Number;
  // }

  // Postion fields
  payload: {
    orientation: {
      w: number;
      x: number;
      y: number;
      z: number;
    },
    position: {
      x: number;
      y: number;
      z: number;
    }
  }

  computed: {
    roll: Number;
    pitch: Number;
    yaw: Number;
  }

  constructor(private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  // starting hook
  ngOnInit() {


    this.position_sub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/position_pose',
      messageType: 'geometry_msgs/Pose'
    });

    this.wheels_sub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/driveFb"', //may switch to odrive
      messageType: 'std_msgs/Float32MultiArray'
    });

    // starts the liistener
    this.wheel_listen();
    this.position_listen();
  }

  // functions

  // angles are in radians
  euler_from_quaternion(x: number, y: number, z: number, w: number) {
    let t0 = +2.0 * (w * x + y * z);
    let t1 = +1.0 - 2.0 * (x * x + y * y);
    let roll_x = Math.atan2(t0, t1);
    let t2 = +2.0 * (w * y - z * x);
    t2 = (t2 > +1.0) ? +1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;
    let pitch_y = Math.asin(t2);
    let t3 = +2.0 * (w * z + x * y);
    let t4 = +1.0 - 2.0 * (y * y + z * z);
    let yaw_z = Math.atan2(t3, t4);

    return [roll_x, pitch_y, yaw_z];
  }

  wheel_listen() {
    this.wheels_sub.subscribe((message: any) => {
      console.log('Received message on ' + this.wheels_sub.name + ': ' + message.data);
    });

  }

  position_listen() {
    this.position_sub.subscribe((message: any) => {
      this.payload = message;
      this.msgstr = JSON.stringify(message);

      let [roll, pitch, yaw] = this.euler_from_quaternion(this.payload.orientation.x, this.payload.orientation.y, this.payload.orientation.z, this.payload.orientation.w);
      this.computed = { roll, pitch, yaw };
      this.cmpstr = JSON.stringify(this.computed);

    });
  }
}

