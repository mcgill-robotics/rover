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
  showModal : boolean = false;

  markerDict : { [key: string]: L.Marker } = {}; // to keep track of markers and debris aread on the map 
  debrisDict : { [key: string]: L.Circle } = {};

  constructor(private markerService: MarkerService, private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  private initMap(): void {
    this.map = L.map('map', {
      attributionControl: false,
    });

    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 17,
      minZoom: 6,
    }).addTo(this.map);
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
    this.markerService.addDefaultObjectiveMarkers(this.map);
    this.markerService.addDefaultDebrisAreas(this.map);
    this.markerService.makeRoverMarker(this.map);
    this.markerService.makeControlStationMarker(this.map);

    this.gps_subscriber.subscribe((message: ROSLIB.Message) => {
      this.markerService.set_gps_data(message);
      this.markerService.moveRoverMarker(this.map);
    });
  }

  addLandmark(name: string, lat: string, lon: string) {
    this.markerDict[name] = this.markerService.makeNewObjectiveMarkers(this.map, name, Number(lat), Number(lon));
  }

  addDebrisArea(name: string, lat: string, lon: string, radius: string) {
    this.debrisDict[name] = this.markerService.makeNewDebrisAreas(this.map, name, Number(lat), Number(lon), Number(radius));
  }

  removeMarker(name: string) {
    alert("Removing marker: " + name);
    this.map.removeLayer(this.markerDict[name]);
    delete this.debrisDict[name];
    console.log(this.debrisDict);
  }
}
