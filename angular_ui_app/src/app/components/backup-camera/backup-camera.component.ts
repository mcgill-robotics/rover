import { Component, ElementRef, Input, ViewChild } from '@angular/core';
import * as ROSLIB from 'roslib';
import { RosService } from 'src/app/ros.service';

@Component({
  selector: 'app-backup-camera',
  templateUrl: './backup-camera.component.html',
  styleUrls: ['./backup-camera.component.scss']
})
export class BackupCameraComponent {
  @ViewChild('imageDisplay') imageDisplay: ElementRef<HTMLImageElement>;
  @Input() cameraTopic: string;
  ros: ROSLIB.Ros;

  constructor(private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit(): void {
    var imageSubscriber = new ROSLIB.Topic({
      ros: this.ros,
      name: this.cameraTopic,
      messageType: 'sensor_msgs/Image'
    });

    imageSubscriber.subscribe((message: any) => {
      this.imageDisplay.nativeElement.src = 'data:image/jpeg;base64,' + message.data;
    });
  }
}
