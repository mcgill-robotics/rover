import { Component } from '@angular/core';
import * as ROSLIB from 'roslib';
import { RosService } from 'src/app/ros.service';

@Component({
  selector: 'app-antenna-feed',
  templateUrl: './antenna-feed.component.html',
  styleUrls: ['./antenna-feed.component.scss']
})
export class AntennaFeedComponent {
  //  ROS fields
  ros: ROSLIB.Ros;
  sub: ROSLIB.Topic; //subscriber
  pub: ROSLIB.Topic; //pub 

  antennaInstruction: ROSLIB.Message; //payload to be sent
  feed: string;
  
  constructor(private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit() {

    this.pub = new ROSLIB.Topic({
      ros : this.ros,
      name : '/test_topic', 
      messageType : 'std_msgs/Float32MultiArray'
    });

    this.sub = new ROSLIB.Topic({
      ros : this.ros,
      name : '/test_topic',
      messageType : 'std_msgs/Float32MultiArray'
    });

    // this.antennaInstruction = new ROSLIB.Message({
    //   data : [0, 0]
    // });

    // starts the liistener
    this.listen()
  }

  listen() {
    this.sub.subscribe((message:any) => {
      console.log(message.data);
      this.pub.publish(new ROSLIB.Message({
        // data : message.data 
        data : message.data 
      }))
      this.feed = message.data;
    });

    }

}
