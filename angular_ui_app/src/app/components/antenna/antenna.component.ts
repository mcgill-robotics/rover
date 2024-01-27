import { Component } from '@angular/core';
import * as ROSLIB from 'roslib';
import { RosService } from 'src/app/ros.service';

@Component({
  selector: 'app-antenna',

  templateUrl: './antenna.component.html',
  styleUrls: ['./antenna.component.scss']
})
export class AntennaComponent {
// ROS fields
  ros: ROSLIB.Ros;
  sub: ROSLIB.Topic; //subscriber
  pub: ROSLIB.Topic; //pub 

  // Page fields
  latitude: number = 0;
  longitude: number = 0;
  antennaInstruction: ROSLIB.Message; //payload to be sent
  payload: string = ""; //payload sent validation
  
  constructor(private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit() {

    this.pub = new ROSLIB.Topic({
      ros : this.ros,
      name : '/antennaData',
      messageType : 'std_msgs/Float32MultiArray'
    });

    this.sub = new ROSLIB.Topic({
      ros : this.ros,
      name : '/antennaData',
      messageType : 'std_msgs/Float32MultiArray'
    });

    this.antennaInstruction = new ROSLIB.Message({
      data : [0, 0]
    });

    // starts the liistener
    this.listen()
  }
  
  publish(payload: string) {
    this.pub.publish({data: payload});
  }

  listen() {
    this.sub.subscribe((message:any) => {
      console.log('Received message on ' +this.sub.name + ': ' + message.data);
      this.payload = ('Received message on ' + this.sub.name +" and the message.data: "+  message.data);
    });

    }

    sendInstruction() {
      this.antennaInstruction = {data: [this.latitude,this.longitude]};
      this.pub.publish(this.antennaInstruction);
    }

}


