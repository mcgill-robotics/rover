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
  private gps_data: number[] = [0, 0];
  private prev_gps_data: number[] = [0, 0];
  private control_station: number[] = [0, 0];
  private blueIcon: L.Icon;
  private redIcon: L.Icon;
  private blackIcon: L.Icon;
  private roverMarker: L.Marker;

  constructor(private http: HttpClient) { 
    this.blueIcon = new L.Icon({
      iconUrl: 'assets/map-pins/rover-location-icon.png',
      iconSize: [41, 41], 
      iconAnchor: [20, 20], 
      popupAnchor: [1, -34]
    });
    
    this.redIcon = new L.Icon({
      iconUrl: 'assets/map-pins/red-map-pin.png',
      iconSize: [41, 41],
      iconAnchor: [20, 39],
      popupAnchor: [1, -34],
      shadowUrl: 'assets/map-pins/marker-shadow.png',
      shadowAnchor: [12, 42]
    });

    this.blackIcon = new L.Icon({
      iconUrl: 'assets/map-pins/black-map-pin.png',
      iconSize: [41, 41],
      iconAnchor: [20, 39],
      popupAnchor: [1, -34],
      shadowUrl: 'assets/map-pins/marker-shadow.png',
      shadowAnchor: [12, 42]
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
        area.setStyle({color: 'orange'});
        area.addTo(map).bindPopup(`Debris area: ${lat}, ${lon}`);
      }
    });
  }

  makeRoverMarker(map: L.Map): void { 
    const rover  = L.latLng([this.gps_data[0], this.gps_data[1]]);
    this.roverMarker = L.marker(rover, {icon: this.blueIcon, rotationAngle: 90, title: "Rover" + " : "+this.gps_data[0] +", "+this.gps_data[1]});
    this.roverMarker.addTo(map);
    map.setView(rover, 1);
  }

  makeControlStationMarker(map: L.Map): void { 
    var control_station = L.latLng([this.control_station[0], this.control_station[1]]);
    L.marker(control_station, {icon: this.blackIcon, title: "Control station" + " : "+this.control_station[0] +", "+this.control_station[1]}).addTo(map);
  }

  set_gps_data(msg: any): void {
    this.gps_data = msg.data;
  }

  moveRoverMarker(map: L.Map): void {
    this.roverMarker.setLatLng([this.gps_data[0], this.gps_data[1]]);
    map.setView([this.gps_data[0], this.gps_data[1]]);
  }
}