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
      // url: 'ws://192.168.154.187:9090'
      url: 'ws://192.168.1.69:9090'
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
