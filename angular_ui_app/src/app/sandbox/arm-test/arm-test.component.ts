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
  data: any;
  armBrushedSub: ROSLIB.Topic;

  constructor(private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit(): void {
    this.armBrushedSub = new ROSLIB.Topic({
      ros: this.ros,
      name: "armBrushedFB",
      messageType: 'std_msgs/Float32MultiArray'
    });

    this.armBrushedSub.subscribe((message: any) => {
      this.data = message.data;
    });
  }
}
