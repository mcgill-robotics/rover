import { Injectable } from '@angular/core';
import * as ROSLIB from 'roslib';
import { MarkerService } from './marker.service';

@Injectable({
  providedIn: 'root'
})
export class RosService {
  private ros: ROSLIB.Ros;
 

  constructor(private markerService: MarkerService) {
    this.ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    });

    this.ros.on('connection', () => {
      this.ros.getTopics(
        (result) => {
          console.log(result);
        }
      );
    });

    var gps_subscriber = new ROSLIB.Topic({
      ros: this.ros,
      name: '/roverGPSData',
      messageType: 'Float32MultiArray'
    })

    gps_subscriber.subscribe(function(message) {
      markerService.set_gps_data(message);
    })
  }

  getRos(): ROSLIB.Ros {
    return this.ros;
  }

  
}