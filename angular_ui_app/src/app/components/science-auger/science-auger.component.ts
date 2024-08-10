import { Component } from '@angular/core';
import * as ROSLIB from 'roslib';
import { RosService } from 'src/app/ros.service';

@Component({
  selector: 'app-science-auger',
  templateUrl: './science-auger.component.html',
  styleUrls: ['./science-auger.component.scss']
})
export class ScienceAugerComponent {

  orientationCounter: number = 0;
  augerTopic: ROSLIB.Topic;
  carouselTopic: ROSLIB.Topic;

  rotationAngle: any = 0;

  ros: ROSLIB.Ros;

  constructor(private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit() {
    this.augerTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: "/augerCmd", //to be changed to proper topic name
      messageType: 'std_msgs/Float64MultiArray'
    });

    this.carouselTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: "/stepperCmd", //to be changed to proper topic name
      messageType: 'std_msgs/Float64MultiArray'
    });
  }

  // Carousel control [up/down, right left]
  turn() {
    this.carouselTopic.publish(new ROSLIB.Message({
      data : [parseFloat(parseFloat(this.rotationAngle).toFixed())] //actual data to be added (direction, )
    }))
  }

  // auger control []
  //may change depending on electrical implementation
  sendAugerInstruction(direction: number[]) {
    // console.log(direction);
    this.augerTopic.publish(new ROSLIB.Message({
      data : direction
    }))
  }
}
