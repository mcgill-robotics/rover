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
  twist: ROSLIB.Message; //payload type
  cmdList: ROSLIB.Topic; 
  cmdVel: ROSLIB.Topic; //request 
  listener: ROSLIB.Topic; //response
  myDisplay: string;

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
  
    // this.cmdList = new ROSLIB.Topic({
    //   ros : this.ros,
    //   name : '/turtle1/cmd_vel',
    //   messageType : 'geometry_msgs/Twist'
    // });

    // this.cmdList.subscribe((message:any) => {
    //   // console.log('Received message on ' +this.listener.name + ': ' + message.y + message.x);
    //   this.myDisplay2 = ('Received message on ' +this.cmdList.name + 'linear : ' + message.linear.x +"and angular: " + message.angular.z);
    // });

    // instantiate a msg 
    this.twist = new ROSLIB.Message({
      linear : {
        x : 0.0,
        y : 0.0,
        z : 0.0
      },
      angular : {
        x : 0.0,
        y : 0.0,
        z : 0.0
      }
    });
    this.listen();
  }

  // publish the msg to the predefined Topic
  publish() {
    this.cmdVel.publish(this.twist);
  }

  foward() {
    this.twist =  {linear : {
      x : 1.0,
      y : 0.0,
      z : 0.0
    },
    angular : {
      x : 0.0,
      y : 0.0,
      z : 0.0
    }}
  }


//   back() {
//     this.twist.linear.x =  -1.0;
// }

  right() {
    this.twist =  {linear : {
      x : 0.0,
      y : 0.0,
      z : 0.0
    },
    angular : {
      x : 0.0,
      y : 0.0,
      z : -1.0
    }}
  }

  left() {
    this.twist =  {
      linear : {
      x : 0.0,
      y : 0.0,
      z : 0.0
    },
    angular : {
      x : 0.0,
      y : 0.0,
      z : 1.0
    }}
  }


  turtleMotion(event:any) {
  console.log(event.key)

switch(event.key) {
  case "w":
    this.foward();
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


  listen() {
  this.listener.subscribe((message:any) => {
    // console.log('Received message on ' +this.listener.name + ': ' + message.y + message.x);
    this.myDisplay = ('Received message on ' + this.listener.name + ' y : ' + message.y +"and x: " + message.x);
  });
  }

}
