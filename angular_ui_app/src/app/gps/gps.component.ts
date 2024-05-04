import { Component, AfterViewInit } from '@angular/core';
import * as L from 'leaflet';
import { MarkerService } from '../marker.service';

@Component({
  selector: 'app-gps',
  templateUrl: './gps.component.html',
  styleUrls: ['./gps.component.scss']
})
export class GpsComponent implements AfterViewInit {

  private map: L.Map;

  constructor(private markerService: MarkerService) { }

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
  }

}
