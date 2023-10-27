import { Component, ElementRef, ViewChild } from '@angular/core';
import { RosService } from '../ros.service';
import * as ROSLIB from 'roslib';

@Component({
  selector: 'app-main-frame',
  templateUrl: './main-frame.component.html',
  styleUrls: ['./main-frame.component.scss']
})
export class MainFrameComponent {
  @ViewChild('rosBridgeMainDiv') rosBridgeMainDiv: ElementRef<HTMLDivElement>;
  rosBridgeStatus: string;
  ros: ROSLIB.Ros;

  constructor(private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit(): void {
    this.ros.on('connection', () => {
      this.rosBridgeStatus = 'Connected to ROSbridge';
      this.rosBridgeMainDiv.nativeElement.className = 'alert alert-primary';
    });

    this.ros.on('error', (error) => {
      this.rosBridgeStatus = `ROSBridgeError: ${error}`;
      this.rosBridgeMainDiv.nativeElement.className = 'alert alert-danger';
    });

    this.ros.on('close', () => {
      this.rosBridgeStatus = 'Connection closed';
      this.rosBridgeMainDiv.nativeElement.className = 'alert alert-warning';
    });
  }
}

