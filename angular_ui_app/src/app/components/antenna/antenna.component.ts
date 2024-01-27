import { Component } from '@angular/core';
import * as ROSLIB from 'roslib';
import { RosService } from 'src/app/ros.service';

@Component({
  selector: 'app-antenna',

  templateUrl: './antenna.component.html',
  styleUrls: ['./antenna.component.scss']
})
export class AntennaComponent {

  ros: ROSLIB.Ros;
  sub: ROSLIB.Topic; //subscriber
  // sub2: ROSLIB.Topic; //subscriber
  pub: ROSLIB.Topic; //pub 

  // payload2: string = "";
  payload: string = "";
  latitude: number = 0;
  longitude: number = 0;
  antennaInstruction: ROSLIB.Message; //payload type
  // antennaInstruction: string;
  
  constructor(private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit() {

    this.pub = new ROSLIB.Topic({
      ros : this.ros,
      name : '/antennaData',
      messageType : 'std_msgs/Float32MultiArray'
      //       name : '/lol',
      // messageType : 'std_msgs/String'
    });

    this.sub = new ROSLIB.Topic({
      ros : this.ros,
      // name : '/chatter',
      // messageType : 'std_msgs/String'
      name : '/antennaData',
      messageType : 'std_msgs/Float32MultiArray'
    });

    this.antennaInstruction = new ROSLIB.Message({
      data : [0, 0]
    });

    // this.sub2 = new ROSLIB.Topic({
    //   ros : this.ros,
    //   name : '/lol',
    //   messageType : 'std_msgs/String'
    //   // name : '/antennaData',
    //   // messageType : 'std_msgs/Float32MultiArray'
    // });

    this.listen()
  }
  
  publish(payload: string) {
    this.pub.publish({data: payload});
  }

  listen() {
    this.sub.subscribe((message:any) => {
      console.log('Received message on ' +this.sub.name + ': ' + message.data);
      this.payload = ('Received message on ' + this.sub.name +"and the message.data"+  message.data);
      // this.publish(this.payload);
      
    });

    // this.sub2.subscribe((message:any) => {
    //   // console.log('Received message on ' +this.listener.name + ': ' + message.y + message.x);
    //   this.payload2 = ('Received message on ' + this.sub2.name +"and the message.data"+  message.data);
      
    // });
    }

    sendInstruction() {
      this.antennaInstruction = JSON.stringify({data: [this.latitude,this.longitude]});
      this.pub.publish(this.antennaInstruction);
    }

}


