import { Component, OnInit } from '@angular/core';
import * as ROSLIB from 'roslib';
import { count } from 'rxjs';
import { RosService } from 'src/app/ros.service';
import { ScienceService } from 'src/app/service/science.service';

@Component({
  selector: 'app-science-sensors',
  templateUrl: './science-sensors.component.html',
  styleUrls: ['./science-sensors.component.scss']
})

export class ScienceSensorsComponent implements OnInit {
  // data: string;
  ros: ROSLIB.Ros;

  // mock topics from electrical
  dataTopic: ROSLIB.Topic;
  geigerTopic: ROSLIB.Topic;
  orientationTopic: ROSLIB.Topic;
  orientationCounter: number = 0;

  // data array field in front end
  // g1, g2, g3, g4, m1, m, m, etc
  d: number[][] = [[], [], [], [], [], [], [], [], [], [], [], []];



  constructor(private scienceService: ScienceService, private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit() {
    this.dataTopic = new ROSLIB.Topic({
      ros: this.ros,
      // name: "/science_data", //to be changed to proper topic name
      name: "/test_topic", //to be changed to proper topic name
      messageType: 'std_msgs/Float32MultiArray'
    });

    // this.geigerTopic = new ROSLIB.Topic({
    //   ros: this.ros,
    //   name: "/test_topic2", //to be changed to proper topic name
    //   messageType: 'std_msgs/Float32MultiArray'
    // });

    this.orientationTopic = new ROSLIB.Topic({
      ros: this.ros,
      // name: "/test_topic", //to be changed to proper topic name
      name: "/stepper_position", //to be changed to proper topic name
      messageType: 'std_msgs/Float32MultiArray'
    });

    this.listen();
  }

  listen() {
    // get the orientation of the carousel
    this.orientationTopic.subscribe((message: any) => {
      // console.log(message);
      this.orientationCounter = message.data;
      }
    )
      
    this.dataTopic.subscribe((message: any) => {
      for (let k = 0; k < message.data.length; k++) {
                if (k < 4 && (k * 2 != this.orientationCounter)) { //must be diagonal to update geiger
          continue //skips other geiger
        }  
        // using concat to create new array for ngOnchange to work in graph
        this.d[k] = this.d[k].concat(message.data[k]);
        // console.log(this.d);
        // Saving data to store
        this.scienceService.storeData(message.data[k], k);
      }
    })
    // this.dataTopic.subscribe((message: any) => {
    //   for (let k = 4; k < message.data.length + 4; k++) {
    //     // using concat to create new array for ngOnchange to work in graph
    //     this.d[k] = this.d[k].concat(message.data[k - 4]);
    //     // Saving data to store
    //     this.scienceService.storeData(message.data[k - 4], k);
    //   }
    // })

    // this.geigerTopic.subscribe((message: any) => {
    //   for (let k = 0; k < 4; k++) {
    //     // if ((k * 2 != this.orientationCounter)) { //must be diagonal to update geiger
    //     //   continue //skips other geiger
    //     // }
    //     this.d[k] = this.d[k].concat(message.data[k]);
    //     this.scienceService.storeData(message.data[k], k);
    //   }
    // })
  }
}
