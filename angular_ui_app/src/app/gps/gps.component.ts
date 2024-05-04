import { Component, AfterViewInit } from '@angular/core';
import * as L from 'leaflet';
import { MarkerService } from '../marker.service';
import { RosService } from '../ros.service';
import * as ROSLIB from 'roslib';

@Component({
  selector: 'app-gps',
  templateUrl: './gps.component.html',
  styleUrls: ['./gps.component.scss']
})
export class GpsComponent implements AfterViewInit {

  private map: L.Map;
  private ros: ROSLIB.Ros;

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

  ngAfterViewInit(): void {
    this.initMap();
    this.markerService.makeObjectiveMarkers(this.map);
    this.markerService.makeDebrisAreas(this.map);
    this.markerService.makeRoverMarker(this.map);
    this.markerService.makeControlStationMarker(this.map);

    var gps_subscriber = new ROSLIB.Topic({
      ros: this.ros,
      name: '/roverGPSData',
      messageType: 'Float32MultiArray'
    })

    gps_subscriber.subscribe((message: ROSLIB.Message) => {
      this.markerService.set_gps_data(message);
    })
  }
}
