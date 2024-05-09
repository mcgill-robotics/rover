import { Component, OnInit } from '@angular/core';
import * as ROSLIB from 'roslib';
import { count } from 'rxjs';
import { RosService } from 'src/app/ros.service';
import { ScienceService } from 'src/app/service/science.service';

@Component({
  selector: 'app-science-page',
  templateUrl: './science-page.component.html',
  styleUrls: ['./science-page.component.scss']
})

export class SciencePageComponent implements OnInit{
  // data: string;
  augerTopic: ROSLIB.Topic;
  carouselTopic: ROSLIB.Topic;
  ros: ROSLIB.Ros;

  // mock topics from electrical
  dataTopic: ROSLIB.Topic;
  geigerTopic: ROSLIB.Topic;

  orientationCounter: number = 0;

  // data array field in front end
  // g1, g2, g3, g4, m1, m, m, etc
  d: number[][] = [[0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0]];
  
  
  
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

    this.dataTopic = new ROSLIB.Topic({
      ros : this.ros,
      name : "/test_topic", //to be changed to proper topic name
      messageType : 'std_msgs/Float32MultiArray'
    });

    this.geigerTopic = new ROSLIB.Topic({
      ros : this.ros,
      name : "/test_topic2", //to be changed to proper topic name
      messageType : 'std_msgs/Float32MultiArray'
    });

    this.listen();
  }

  listen() {
    this.dataTopic.subscribe((message:any) => {
      for (let k = 4; k < message.data.length + 4; k++) {
        // using concat to create new array for ngOnchange to work in graph
        this.d[k] = this.d[k].concat(message.data[k-4]);
        // Saving data to store
        this.scienceService.storeData(message.data[k-4], k);
      }  
    })

    this.geigerTopic.subscribe((message:any) => {
      for (let k = 0; k < 4; k++) {
        if ((k*2 != this.orientationCounter)) { //must be diagonal to update geiger
          continue //skips other geiger
        }
        this.d[k] = this.d[k].concat(message.data[k]);
        this.scienceService.storeData(message.data[k], k);
      }  
    })    


  }

  
  // Carousel control
  turn() {
    this.orientationCounter != 7 ? this.orientationCounter++: this.orientationCounter = 0;

    this.carouselTopic.publish(new ROSLIB.Message({
      data : [1, 0] //actual data to be added (direction, )
    }))
  }

  // auger control 
  //may change depending on electrical implementation
  sendAugerInstruction(direction: number[]) {
    // console.log(direction);
    this.augerTopic.publish(new ROSLIB.Message({
      data : direction
    }))
  }

}
