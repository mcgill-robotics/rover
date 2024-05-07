import { Component, Input} from '@angular/core';
import * as ROSLIB from 'roslib';
import { RosService } from 'src/app/ros.service';


@Component({
  selector: 'app-arm-component',
  templateUrl: './arm.component.html',
  styleUrls: ['./arm.component.scss']
})
export class ArmComponent {
  ros: ROSLIB.Ros;     
  arm_sub_brushed: ROSLIB.Topic;
  arm_sub_brushless: ROSLIB.Topic;

  @Input() initialValue: number = 0;

  // array contains initial values 
  array  : number[]= [0,0,0];

  armBrushed: {
    wrist: number[];
    elbow: Number[];
    shoulder: Number[];
  } = {
    wrist: [0],
    elbow: [0],
    shoulder: [0]
  };

  armBrushless: {
    wrist: number[];
    elbow: Number[];
    shoulder: Number[];
  } = {
    wrist: [0],
    elbow: [0],
    shoulder: [0]
  };

  increment(id: number) {
    console.log("id: ", id);
    this.array[id] += 0.5;
    console.log("val 0: ", this.array[id]);
  }

  decrement(id: number) {
    this.array[id] -= 0.5;
  }

  constructor(private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit() {
    this.arm_sub_brushed = new ROSLIB.Topic({
      ros: this.ros,
      name: '/armBrushedFB',
      messageType: 'std_msgs/Float32MultiArray'
    });

    this.arm_sub_brushless = new ROSLIB.Topic({
      ros: this.ros,
      name: '/armBrushlessFB',
      messageType: 'std_msgs/Float32MultiArray'
    });

    this.arm_listen();

  }
  arm_listen() {
    this.arm_sub_brushed.subscribe((message: any) => {
      this.armBrushed.wrist = message.data[0].toFixed(3);
      this.armBrushed.elbow = message.data[1].toFixed(3);
      this.armBrushed.shoulder = message.data[2].toFixed(3);
    })

    this.arm_sub_brushless.subscribe((message: any) => {
      this.armBrushless.wrist = message.data[0].toFixed(3);
      this.armBrushless.elbow = message.data[1].toFixed(3);
      this.armBrushless.shoulder = message.data[2].toFixed(3);
    })
  }
}

