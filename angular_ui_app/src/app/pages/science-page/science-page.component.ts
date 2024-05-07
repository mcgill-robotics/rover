import { Component } from '@angular/core';
import * as ROSLIB from 'roslib';
import { RosService } from 'src/app/ros.service';
import { ScienceService } from 'src/app/service/science.service';

@Component({
  selector: 'app-science-page',
  templateUrl: './science-page.component.html',
  styleUrls: ['./science-page.component.scss']
})
export class SciencePageComponent {

  data: string;
  augerTopic: ROSLIB.Topic;
  carouselTopic: ROSLIB.Topic;
  ros: ROSLIB.Ros;
  
  constructor(private scienceService: ScienceService, private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit() {
    this.augerTopic = new ROSLIB.Topic({
      ros : this.ros,
      name : "topicName", //to be changed to proper topic name
      messageType : 'std_msgs/Float32MultiArray'
    });

    this.carouselTopic = new ROSLIB.Topic({
      ros : this.ros,
      name : "topicName", //to be changed to proper topic name
      messageType : 'std_msgs/Float32MultiArray'
    });
  }

  // carousel
  turn() {
    this.carouselTopic.publish(new ROSLIB.Message({
      data : [1, 0] //actual data to be added
    }))
  }

  // auger control //may change depending on electrical implementation
  sendAugerInstruction(direction: any[]) {
    this.augerTopic.publish(new ROSLIB.Message({
      data : direction //actual data to be added
    }))
  }

  // up() {
  //   this.augerTopic.publish(new ROSLIB.Message({
  //     data : [1, 0] //actual data to be added
  //   }))
  // }

  // down() {
  //   this.augerTopic.publish(new ROSLIB.Message({
  //     data : [1, 0] //actual data to be added
  //   }))
  // }
  
  // left() {
  //   this.augerTopic.publish(new ROSLIB.Message({
  //     data : [1, 0] //actual data to be added
  //   }))
  // }
  
  // right () {
  //   this.augerTopic.publish(new ROSLIB.Message({
  //     data : [1, 0] //actual data to be added
  //   }))
  // }

  // start () {
  //   this.augerTopic.publish(new ROSLIB.Message({
  //     data : [1, 0] //actual data to be added
  //   }))
  // }

  // stop () {
  //   this.augerTopic.publish(new ROSLIB.Message({
  //     data : [1, 0] //actual data to be added
  //   }))
  // }


  collect() {
    this.data = this.scienceService.getData().toString();
  }
  add() {
    this.scienceService.addData("halo");
    this.scienceService.storeData(123, 2, 0);
  }
}
