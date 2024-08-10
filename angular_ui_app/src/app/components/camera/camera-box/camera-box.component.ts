import { Component, Input } from '@angular/core';
import { RosService } from 'src/app/ros.service';
import * as ROSLIB from 'roslib';

@Component({
  selector: 'app-camera-box',
  templateUrl: './camera-box.component.html',
  styleUrls: ['./camera-box.component.scss']
})
export class CameraBoxComponent {
  isHovered: boolean = true;
  ros: ROSLIB.Ros;

  @Input() topic : string;
  @Input() size : string;

  public imageSrc: string;

  constructor(
    private rosService: RosService
  ) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit() {
    new ROSLIB.Topic({
      ros: this.ros,
      name: this.topic, //'/usb_cam/image_raw/compressed',
      messageType: 'sensor_msgs/Image'
    }).subscribe((message: any) => {
      this.imageSrc = 'data:image/jpeg;base64,' + message.data;
    });
  }
}