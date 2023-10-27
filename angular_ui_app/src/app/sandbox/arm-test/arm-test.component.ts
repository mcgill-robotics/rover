import { Component } from '@angular/core';
import * as ROSLIB from 'roslib';
import { RosService } from 'src/app/ros.service';

@Component({
  selector: 'app-arm-test',
  templateUrl: './arm-test.component.html',
  styleUrls: ['./arm-test.component.scss']
})
export class ArmTestComponent {
  ros: ROSLIB.Ros;
  headerList: string[] = ["Waist", "Shoulder", "Elbow", "Wrist", "Disk", "EOAT"];
  headerValue: any[] = [0,0,0,0,0,0];
  data: any; //[EOAT, dist, wrist]
  data2: any;
  data3: any; /* msg: ArmControllerInput
              float32 X_dir
              float32 Y_dir
              float32 Z_dir
              float32 MaxVelPercentage
              int32   Mode
              */
  data4: any;

  arm_brushed_sub: ROSLIB.Topic;
  arm_brushless_sub: ROSLIB.Topic;
  arm_control_sub: ROSLIB.Topic;
  arm_error_sub: ROSLIB.Topic;

  constructor(private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit(): void {

    // arm brushed --will be stored in a specific function
    this.arm_brushed_sub = new ROSLIB.Topic({
      ros: this.ros,
      name: "armBrushedFB",
      messageType: 'std_msgs/Float32MultiArray'
    });

    this.arm_brushed_sub.subscribe((message: any) => {
      this.data = message.data;
    });

      // arm brushless
      this.arm_brushless_sub = new ROSLIB.Topic({
      ros: this.ros,
      name: "armBrushlessFB",
      messageType: 'std_msgs/Float32MultiArray'
    });

    this.arm_brushless_sub.subscribe((message: any) => {
      this.data2 = message.data;
    });

      // arm controller input
      this.arm_control_sub = new ROSLIB.Topic({
      ros: this.ros,
      name: "arm_controller_input",
      messageType: 'ArmControllerInput'
    });

    this.arm_control_sub.subscribe((message: any) => {
      this.data3 = message.data;
    });

      // arm error listener 
      this.arm_error_sub = new ROSLIB.Topic({
      ros: this.ros,
      name: "armError",
      messageType: 'String'
    });

    this.arm_error_sub.subscribe((message: any) => {
      this.data4 = message.data;// may need to switch to simple data
    });

  }
}


/* 
Concern: foward kinematics
Use JS to fully implement on the front end
Use the Python backend and make Ajax requests
get the data from the current running UI?


*/
