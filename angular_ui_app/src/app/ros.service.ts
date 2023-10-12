import { Injectable } from '@angular/core';
import * as ROSLIB from 'roslib';

@Injectable({
  providedIn: 'root'
})
export class RosService {
  private ros: ROSLIB.Ros;

  constructor() {
    this.ros = new ROSLIB.Ros({
      url: 'ws://192.168.0.100:9090' //do not activating port forwarding 
      // user ng serve --host 0.0.0.0
      // works over lan, must later be tested with an access point without wifi
      
      // url: 'ws://localhost:9090'
      
    });

    this.ros.on('connection', () => {
      this.ros.getTopics(
        (result) => {
          console.log(result);
        }
      );
    });
  }

  getRos(): ROSLIB.Ros {
    return this.ros;
  }
}