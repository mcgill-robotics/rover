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
  private gps_data: number[] = [45.506103790, -73.57566751708];
  private blueIcon: L.Icon;
  private redIcon: L.Icon;
  private blackIcon: L.Icon;

  constructor(private http: HttpClient) { 
    this.blueIcon = new L.Icon({
      iconUrl: 'assets/map-pins/blue-map-pin.png',
      iconSize: [41, 41], 
      iconAnchor: [12, 41], 
      popupAnchor: [1, -34],
      shadowUrl: 'assets/map-pins/marker-shadow.png',
      shadowAnchor: [5, 42]
    });
    
    this.redIcon = new L.Icon({
      iconUrl: 'assets/map-pins/red-map-pin.png',
      iconSize: [41, 41],
      iconAnchor: [12, 41],
      popupAnchor: [1, -34],
      shadowUrl: 'assets/map-pins/marker-shadow.png',
      shadowAnchor: [5, 42]
    });

    this.blackIcon = new L.Icon({
      iconUrl: 'assets/map-pins/black-map-pin.png',
      iconSize: [41, 41],
      iconAnchor: [12, 41],
      popupAnchor: [1, -34],
      shadowUrl: 'assets/map-pins/marker-shadow.png',
      shadowAnchor: [5, 42]
    });
  }

  makeDebrisMarkers(map: L.Map): void { 
    this.http.get(this.debris).subscribe((res: any) => {
      for (const c of res.features) {
        const lon = c.geometry.coordinates[1];
        const lat = c.geometry.coordinates[0];
        const marker = L.marker([lat, lon], {icon: this.redIcon});
        marker.addTo(map);
      }
    });
  };

  makeAreas(map: L.Map): void { 
    this.http.get(this.areas).subscribe((res: any) => {
      for (const c of res.features) {
        const lon = c.geometry.coordinates[1];
        const lat = c.geometry.coordinates[0];
        const rad = c.geometry.size;
        const area = L.circleMarker([lat, lon], {radius: rad});
        area.setStyle({color: 'green'});
        area.addTo(map);
      }
    });
  }

  makeRoverMarker(map: L.Map): void { 
    const rover  = L.latLng([this.gps_data[0], this.gps_data[1]]);
    const markerr = L.marker(rover, {icon: this.blueIcon});
    
    markerr.addTo(map);
  }

  makeControlStationMarker(map: L.Map): void { 
    var control_station = L.latLng([45.5056037902832, -73.57576751708984]);
    L.marker(control_station, {icon: this.blackIcon}).addTo(map);
    map.setView(control_station, 1);
  }

  set_gps_data(msg: any): void {
    this.gps_data = msg.data;

  }
}