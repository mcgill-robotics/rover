import { Component, AfterViewInit } from '@angular/core';
import * as L from 'leaflet';
import { MarkerService } from '../marker.service';

const iconRetinaUrl = 'assets/marker-icon-2x.png';
const iconUrl = 'assets/marker-icon.png';
const shadowUrl = 'assets/marker-shadow.png';
const iconDefault = L.icon({
  iconRetinaUrl,
  iconUrl,
  shadowUrl,
  iconSize: [25, 41],
  iconAnchor: [12, 41],
  popupAnchor: [1, -34],
  tooltipAnchor: [16, -28],
  shadowSize: [41, 41]
});
L.Marker.prototype.options.icon = iconDefault;

@Component({
  selector: 'app-map',
  templateUrl: './map.component.html',
  styleUrls: ['./map.component.scss']
})
export class MapComponent implements AfterViewInit{

  private map: L.Map;
  
  constructor(private markerService: MarkerService) { }

  private initMap(): void {
    this.map = L.map('map', {
      crs: L.CRS.Simple,
      minZoom: 15
    });
  }

  ngAfterViewInit(): void { 
    this.initMap();
    this.markerService.makeDebrisMarkers(this.map);
    this.markerService.makeAreas(this.map);
    this.markerService.makeRoverMarker(this.map);
    var control_station = L.latLng([45.5056037902832, -73.57576751708984]);
    L.marker(control_station).addTo(this.map);
    this.map.setView(control_station, 1);

  }

}
