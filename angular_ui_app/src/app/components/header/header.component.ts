import { Component, ElementRef, ViewChild } from '@angular/core';
import { RosService } from '../../ros.service';
import * as ROSLIB from 'roslib';

@Component({
  selector: 'app-header',
  templateUrl: './header.component.html',
  styleUrls: ['./header.component.scss']
})
export class HeaderComponent {
  rosBridgeStatus: string;
  @ViewChild('rosBridgeMainDiv') rosBridgeMainDiv: ElementRef<HTMLDivElement>;
  ros: ROSLIB.Ros;

  constructor(private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  ngOnInit(): void {
    this.ros.on('connection', () => {
      this.rosBridgeStatus = 'Connected to ROSbridge';
      this.rosBridgeMainDiv.nativeElement.style.color = 'green';
    });

    this.ros.on('error', (error) => {
      this.rosBridgeStatus = `ROSBridgeError: ${error}`;
      this.rosBridgeMainDiv.nativeElement.style.color = 'red';
    });

    this.ros.on('close', () => {
      this.rosBridgeStatus = 'Connection closed';
      this.rosBridgeMainDiv.nativeElement.style.color = 'red';
    });
  }
}
