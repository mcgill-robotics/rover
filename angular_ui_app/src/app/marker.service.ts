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
  private gps_data: number[] = [51.458529866888036, -112.70487262267218];
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

  addDefaultObjectiveMarkers(map: L.Map): void {
    this.http.get(this.debris).subscribe((res: any) => {
      for (const c of res.features) {
        const lon = c.geometry.coordinates[1];
        const lat = c.geometry.coordinates[0];
        this.makeNewObjectiveMarkers(map, c.properties.title, lat, lon);
      }
    });
  }

  addDefaultDebrisAreas(map: L.Map): void {

  }

  makeNewObjectiveMarkers(map: L.Map, title: string, lat: number, long: number): L.Marker {
    const marker = L.marker([lat, long], { icon: this.redIcon, title: title + " : " + lat + ", " + long });
    marker.addTo(map);
    return marker;
  };

  makeNewDebrisAreas(map: L.Map, title: string, lat: number, long: number, radius: number): L.Circle {
    const area = L.circle([lat, long], { radius: radius })
    area.setStyle({ color: 'orange' });
    console.log(area);
    area.addTo(map).bindPopup(`Debris area: ${lat}, ${long}`);
    return area;
  }

  makeRoverMarker(map: L.Map): void {
    const rover = L.latLng([this.gps_data[0], this.gps_data[1]]);
    this.roverMarker = L.marker(rover, { icon: this.blueIcon, rotationAngle: 90, title: "Rover" + " : " + this.gps_data[0] + ", " + this.gps_data[1] });
    this.roverMarker.addTo(map);
    map.setView(rover, 16);
  }

  makeControlStationMarker(map: L.Map): void {
    var control_station = L.latLng([this.control_station[0], this.control_station[1]]);
    L.marker(control_station, { icon: this.blackIcon, title: "Control station" + " : " + this.control_station[0] + ", " + this.control_station[1] }).addTo(map);
  }

  set_gps_data(msg: any): void {
    this.gps_data = msg.data;
  }

  moveRoverMarker(map: L.Map): void {
    this.roverMarker.setLatLng([this.gps_data[0], this.gps_data[1]]);
    map.setView([this.gps_data[0], this.gps_data[1]]);
  }
}
