import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import * as L from 'leaflet';
import {MapComponent} from './map/map.component';

@Injectable({
  providedIn: 'root'
})
export class MarkerService {
  debris: string = '/assets/debris-markers.geojson';
  areas: string = '/assets/map-areas.geojson';
  gps_data: number[] = [45.506503790, -73.57566751708];

  constructor(private http: HttpClient) { }

  makeDebrisMarkers(map: L.Map): void { 
    this.http.get(this.debris).subscribe((res: any) => {
      for (const c of res.features) {
        const lon = c.geometry.coordinates[1];
        const lat = c.geometry.coordinates[0];
        const marker = L.marker([lat, lon]);
        marker.addTo(map);
      }
    });
  };

  makeAreas(map: L.Map): void { 
    this.http.get(this.areas).subscribe((res: any) => {
      for (const c of res.features) {
        const lon = c.geometry.coordinates[1];
        const lat = c.geometry.coordinates[0];
        const area = L.circleMarker([lat, lon]);
        
        area.addTo(map);
      }
    });
  }

  makeRoverMarker(map: L.Map): void { 
    const rover  = L.latLng([this.gps_data[0], this.gps_data[1]]);
    const markerr = L.marker(rover);
    markerr.addTo(map);
  }

  set_gps_data(msg: any): void {
    this.gps_data = msg.data;

  }
}