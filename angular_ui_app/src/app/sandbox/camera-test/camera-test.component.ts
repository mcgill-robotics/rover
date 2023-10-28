import { Component } from '@angular/core';
import * as ROSLIB from 'roslib';
import { RosService } from 'src/app/ros.service';

@Component({
  selector: 'app-camera-test',
  templateUrl: './camera-test.component.html',
  styleUrls: ['./camera-test.component.scss']
})
export class CameraTestComponent {
  ros: ROSLIB.Ros;
  cameraTopic = "/camera_frames";
  public imagePath: string;

  constructor(private rosService: RosService) {
    this.ros = this.rosService.getRos();
    this.imagePath = "default";
  }
  
  ngOnInit(): void {
    var imageSubscriber = new ROSLIB.Topic({
      ros: this.ros,
      name: this.cameraTopic,
      messageType: 'sensor_msgs/Image'
    });

    imageSubscriber.subscribe((message: any) => {
      // console.log(message.data);
      this.imagePath = 'data:image/jpeg;base64,' + message.data;

      // create a request to python code
    });
  }
}
