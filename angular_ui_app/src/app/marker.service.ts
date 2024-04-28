import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import * as LeafRotated from 'leaflet-rotatedmarker';
import * as L from 'leaflet';
@Injectable({
  providedIn: 'root'
})
export class MarkerService {
  debris: string = '/assets/obstacle-markers.geojson';
  areas: string = '/assets/debris-areas.geojson';
  private gps_data: number[] = [45.406003790, -73.67566751708];
  private prev_gps_data: number[] = [45.306103790, -73.57566751708];
  private control_station: number[] = [45.8056037902832, -93.27576751708984];
  private blueIcon: L.Icon;
  private redIcon: L.Icon;
  private blackIcon: L.Icon;

  constructor(private http: HttpClient) { 
    this.blueIcon = new L.Icon({
      iconUrl: 'assets/map-pins/direction-arrow.png',
      iconSize: [41, 41], 
      iconAnchor: [12, 41], 
      popupAnchor: [1, -34]
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

  makeObjectiveMarkers(map: L.Map): void { 
    this.http.get(this.debris).subscribe((res: any) => {
      for (const c of res.features) {
        const lon = c.geometry.coordinates[1];
        const lat = c.geometry.coordinates[0];
        const marker = L.marker([lat, lon], {icon: this.redIcon, title: c.properties.name + " : "+lat +", "+lon});
        marker.addTo(map);
      }
    });
  };

  makeDebrisAreas(map: L.Map): void { 
    this.http.get(this.areas).subscribe((res: any) => {
      for (const c of res.features) {
        const lon = c.geometry.coordinates[1];
        const lat = c.geometry.coordinates[0];
        const rad = c.geometry.size;
        const area = L.circle([lat, lon], {radius: rad});
        area.setStyle({color: 'green'});
        area.addTo(map);
      }
    });
  }

  makeRoverMarker(map: L.Map): void { 
    const rover  = L.latLng([this.gps_data[0], this.gps_data[1]]);
    const markerr = L.marker(rover, {icon: this.blueIcon, rotationAngle: 90, title: "Rover" + " : "+this.gps_data[0] +", "+this.gps_data[1]});
    markerr.addTo(map);
    map.setView(rover, 1);
  }

  makeControlStationMarker(map: L.Map): void { 
    var control_station = L.latLng([this.control_station[0], this.control_station[1]]);
    L.marker(control_station, {icon: this.blackIcon, title: "Control station" + " : "+this.control_station[0] +", "+this.control_station[1]}).addTo(map);
  }

  set_gps_data(msg: any): void {
    this.gps_data = msg.data;

  }
}