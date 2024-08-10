import { tileLayerOffline, savetiles } from 'leaflet.offline';
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
  showModal: boolean = false;

  currentCoords: number[] = [0, 0];

  markerDict: { [key: string]: L.Marker } = {}; // to keep track of markers and debris aread on the map 
  debrisDict: { [key: string]: L.Circle } = {};

  constructor(private markerService: MarkerService, private rosService: RosService) {
    this.ros = this.rosService.getRos();
  }

  private initMap(): void {
    this.map = L.map('map', {
      attributionControl: false,
    });

    const baseLayer = tileLayerOffline('http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}', {
      maxZoom: 19,
      minZoom: 13,
      subdomains:['mt0','mt1','mt2','mt3']
    }).addTo(this.map);

    savetiles(baseLayer, {
      zoomlevels: [15, 16, 17, 18, 19], // optional zoomlevels to save, default current zoomlevel
      alwaysDownload: false,
      confirm(layer: any, successCallback: any) {
        // eslint-disable-next-line no-alert
        if (window.confirm(`Save ${layer._tilesforSave.length}`)) {
          successCallback();
        }
      },
      confirmRemoval(layer: any, successCallback: any) {
        // eslint-disable-next-line no-alert
        if (window.confirm('Remove all the tiles?')) {
          successCallback();
        }
      },
      saveText: '<i class="fa fa-download" title="Save tiles"></i>',
      rmText: '<i class="fa fa-trash" title="Remove tiles"></i>',
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

    this.gps_subscriber.subscribe((message: any) => {
      this.markerService.set_gps_data(message);
      this.markerService.moveRoverMarker(this.map);
      this.currentCoords = message.data;
    });

    this.loadMarkersFromStorage();
  }

  addLandmark(name: string, lat: string, lon: string) {
    this.markerDict[name] = this.markerService.makeNewObjectiveMarkers(this.map, name, Number(lat), Number(lon));
    console.log("Added landmark area:", this.markerDict[name]); // ADDED FOR DEBUGGING
    this.saveMarkersToStorage(); // Save markers to localStorage
  }

  addDebrisArea(name: string, lat: string, lon: string, radius: string) {
    this.debrisDict[name] = this.markerService.makeNewDebrisAreas(this.map, name, Number(lat), Number(lon), Number(radius));
    console.log("Added debris area:", this.debrisDict[name]); // ADDED FOR DEBUGGING
    this.saveMarkersToStorage();
  }

  removeMarker(name: string) {
    alert("Removing marker: " + name);

    if (this.markerDict[name]) {
      this.map.removeLayer(this.markerDict[name]);
      delete this.markerDict[name];
    }

    if (this.debrisDict[name]) {
      this.map.removeLayer(this.debrisDict[name]);
      delete this.debrisDict[name];
    }

    this.saveMarkersToStorage();
    console.log(this.debrisDict);
  }

  private saveMarkersToStorage() {
    const markers = Object.keys(this.markerDict).map(key => ({
      name: key,
      lat: this.markerDict[key].getLatLng().lat,
      lng: this.markerDict[key].getLatLng().lng,
    }));

    const debris = Object.keys(this.debrisDict).map(key => ({
      name: key,
      lat: this.debrisDict[key].getLatLng().lat,
      lng: this.debrisDict[key].getLatLng().lng,
      radius: this.debrisDict[key].getRadius(),
    }));

    localStorage.setItem('markers', JSON.stringify(markers)); // array to json 
    localStorage.setItem('debris', JSON.stringify(debris));
  }

  private loadMarkersFromStorage() {
    const markers = JSON.parse(localStorage.getItem('markers') || '[]');
    const debris = JSON.parse(localStorage.getItem('debris') || '[]');

    markers.forEach((marker: { name: string, lat: number, lng: number }) => {
      this.markerDict[marker.name] = this.markerService.makeNewObjectiveMarkers(this.map, marker.name, marker.lat, marker.lng);
    });

    debris.forEach((debris: { name: string, lat: number, lng: number, radius: number }) => {
      this.debrisDict[debris.name] = this.markerService.makeNewDebrisAreas(this.map, debris.name, debris.lat, debris.lng, debris.radius);
    });
  }

}
