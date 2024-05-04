import { Component, AfterViewInit } from '@angular/core';
import * as L from 'leaflet';
import { MarkerService } from 'src/app/marker.service';
import { RosService } from 'src/app/ros.service';
import * as ROSLIB from 'roslib';

@Component({
  selector: 'app-gps',
  templateUrl: './gps.component.html',
  styleUrls: ['./gps.component.scss']
})
export class GpsComponent implements AfterViewInit {

  private map: L.Map;
  private ros: ROSLIB.Ros;
  private gps_subscriber: ROSLIB.Topic;

  constructor(private markerService: MarkerService, private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  private initMap(): void {
    this.map = L.map('map', {
      crs: L.CRS.Simple,
      minZoom: -5,
      maxZoom: 12,
      attributionControl: false,
    });
  }

  ngOnInit() {
    this.gps_subscriber = new ROSLIB.Topic({
      ros: this.ros,
      name: '/roverGPSData',
      messageType: 'std_msgs/Float32MultiArray'
    });
  }

  ngAfterViewInit(): void {
    this.initMap();
    this.markerService.makeObjectiveMarkers(this.map);
    this.markerService.makeDebrisAreas(this.map);
    this.markerService.makeRoverMarker(this.map);
    this.markerService.makeControlStationMarker(this.map);

    this.gps_subscriber.subscribe((message: ROSLIB.Message) => {
      this.markerService.set_gps_data(message);
      this.markerService.moveRoverMarker(this.map);
    });
  }
}
