import { Injectable } from '@angular/core';
import { AppConstants } from './constants';
import * as ROSLIB from 'roslib';

@Injectable({
  providedIn: 'root'
})
export class RosService {
  private ros: ROSLIB.Ros;

  constructor() {
    this.ros = new ROSLIB.Ros({
<<<<<<< HEAD
      url: 'ws://0.0.0.0:9090' //replace with your ip for port forwarding 
      // user ng serve --host 0.0.0.0
      // works over LAN, must later be tested with an access point without wifi
      // url: 'ws://10.122.8.160:9090'
=======
      url: 'ws://' + AppConstants.HOST_IP + ':9090'
>>>>>>> 6f7ca508434d866472d199e57b0bb4593daac748
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
