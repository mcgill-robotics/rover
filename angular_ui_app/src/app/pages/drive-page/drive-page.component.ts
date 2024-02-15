import { Component, ElementRef, ViewChild } from '@angular/core';
import { RosService } from '../../ros.service';
import * as ROSLIB from 'roslib';

@Component({
  selector: 'app-drive-page',
  templateUrl: './drive-page.component.html',
  styleUrls: ['./drive-page.component.scss']
})
export class DrivePageComponent {

  ngOnInit() {}
}
