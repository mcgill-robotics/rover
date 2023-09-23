import { Component } from '@angular/core';
import { RosService } from '../../ros.service';
import * as ROSLIB from 'roslib';

@Component({
  selector: 'app-turtlesim',
  templateUrl: './turtlesim.component.html',
  styleUrls: ['./turtlesim.component.scss']
})
export class TurtlesimComponent {
  rosBridgeStatus: string;
  ros: ROSLIB.Ros;
  cmdVel: ROSLIB.Topic;
  cmdList: ROSLIB.Topic;
  listener: ROSLIB.Topic;
  twist: ROSLIB.Message;
  myDisplay: string;
  myDisplay2: string;


  constructor(private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }


  ngOnInit() {
    this.cmdVel = new ROSLIB.Topic({
      ros : this.ros,
      name : 'turtle1/cmd_vel',
      messageType : 'geometry_msgs/Twist'
    });

    this.listener = new ROSLIB.Topic({
      ros : this.ros,
      name : '/turtle1/pose',
      messageType : 'turtlesim/Pose'
    });
  
    this.cmdList = new ROSLIB.Topic({
      ros : this.ros,
      name : '/turtle1/cmd_vel',
      messageType : 'geometry_msgs/Twist'
    });

    this.cmdList.subscribe((message:any) => {
      // console.log('Received message on ' +this.listener.name + ': ' + message.y + message.x);
      this.myDisplay2 = ('Received message on ' +this.cmdList.name + 'linear : ' + message.linear.x +"and angular: " + message.angular.z);
    });

    this.twist = new ROSLIB.Message({
      linear : {
        x : 0.1,
        y : 0.2,
        z : 0.3
      },
      angular : {
        x : -0.1,
        y : -0.2,
        z : -0.3
      }
    });
  }

  publish() {
    this.cmdVel.publish(this.twist);
  }

  up() {
    this.twist =  {linear : {
      x : 0.5,
      y : 0.0,
      z : 0.0
    },
    angular : {
      x : -0.0,
      y : -0.0,
      z : -0.0
    }}

    // this.cmdVel.publish(this.twist);
  }
  right() {
    this.twist =  {linear : {
      x : 0.0,
      y : 0.0,
      z : 0.0
    },
    angular : {
      x : -0.0,
      y : -0.0,
      z : -0.5
    }}

    // this.cmdVel.publish(this.twist);
  }
  left() {
    this.twist =  {
      linear : {
      x : 0.0,
      y : 0.0,
      z : 0.0
    },
    angular : {
      x : -0.0,
      y : -0.0,
      z : 0.5
    }}

    // this.cmdVel.publish(this.twist);
  }

  turtleMotion(event:any) {
  // console.log(event.key)

switch(event.key) {
  case "w":
    this.up();
    break;
  case "d":
    this.right();
    break;
  case "a":
    this.left();
    break;
}
this.cmdVel.publish(this.twist);

  }


  listen(){
  this.listener.subscribe((message:any) => {
    console.log('Received message on ' +this.listener.name + ': ' + message.y + message.x);
    this.myDisplay = ('Received message on ' +this.listener.name + ' y : ' + message.y +"and x: " + message.x);
  });
  }

}
