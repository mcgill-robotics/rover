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
  data: string;
  augerTopic: ROSLIB.Topic;
  carouselTopic: ROSLIB.Topic;
  ros: ROSLIB.Ros;

  // mock prop data
  dataTopic: ROSLIB.Topic;
  // inject:number[] = [0];
  // c1:number[] = []; //[g, m, h]
  // c2:number[] = [];
  // c3:number[] = [];
  // c4:number[] = [];
  // h1:number[] = []; //[g, m, h]
  // h2:number[] = [];
  // h3:number[] = [];
  // h4:number[] = [];
  // ph1:number[] = []; //[g, m, h]
  // ph2:number[] = [];
  // ph3:number[] = [];
  // ph4:number[] = [];

  orientationCounter: number = 0;

  // g, g, g, g, m, m, m, 
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

    this.listen();
  }

  listen() {
    this.dataTopic.subscribe((message:any) => {
      console.log(message.data);  
      for (let k = 0; k < message.data.length; k++) {
        if (k < 4 && (this.orientationCounter % 2 == 1 || k != 2*this.orientationCounter)) { //must be diagonal to update geiger
          continue //skips a geiger
        }
        this.d[k] = this.d[k].concat(message.data[k]);
      }
        // this.inject = this.inject.concat(message.data[0] as number); //must never pass by ref 
        
    })
  }

  // Control @@@@@@@@@@@@@@@@@@@

  // carousel
  turn() {
    
    this.orientationCounter != 7 ? this.orientationCounter++: this.orientationCounter = 0;

    this.carouselTopic.publish(new ROSLIB.Message({
      data : [1, 0] //actual data to be added (direction, )
    }))
  }

  // auger control //may change depending on electrical implementation
  sendAugerInstruction(direction: any[]) {
    console.log(direction)
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


  // @@@@@@@@@@ Data Persistence 
  collect() {
    this.data = this.scienceService.getData().toString();
  }

  add() {
    this.scienceService.addData("halo");
    this.scienceService.storeData(123, 2, 0);
  }
}
