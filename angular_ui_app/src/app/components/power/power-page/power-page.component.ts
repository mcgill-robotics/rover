import { NoopAnimationPlayer } from '@angular/animations';
import { Component } from '@angular/core';
import * as ROSLIB from 'roslib';
import { RosService } from 'src/app/ros.service';

@Component({
  selector: 'app-power-page',
  templateUrl: './power-page.component.html',
  styleUrls: ['./power-page.component.scss']
})
export class PowerPageComponent {
  rosBridgeStatus: string;
  ros: ROSLIB.Ros;
  String: ROSLIB.Topic; //payload type
  Pub: ROSLIB.Topic; //payload type
  
  Results: string;
    
  constructor(private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit() {

    this.String = new ROSLIB.Topic({
      ros : this.ros,
      name : '/chatter',
      messageType : 'std_msgs/String'
    });

    this.Pub = new ROSLIB.Topic({
      ros : this.ros,
      name : 'lol',
      messageType : 'std_msgs/String'
    });

    this.listen()
  }
  
  publish() {
    this.Pub.publish({data:"YYOYOYY"+this.Results});
  }

  listen() {
    this.String.subscribe((message:any) => {
      // console.log('Received message on ' +this.listener.name + ': ' + message.y + message.x);
      this.Results = ('Received message on ' + this.String.name +"and the message.data"+  message.data);
      this.publish();
    });
    }


  
}

