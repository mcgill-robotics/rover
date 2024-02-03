import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';

import { AppComponent } from './app.component';
import { MainFrameComponent } from './main-frame/main-frame.component';
import { RosService } from './ros.service';
import { HttpClientModule } from '@angular/common/http';
import { MarkerService } from './marker.service';
import { CamerasWidgetComponent } from './cameras-widget/cameras-widget.component';
import { CameraBoxComponent } from './cameras-widget/camera-box/camera-box.component';
import { GpsComponent } from './gps/gps.component';
import { ProcessLogComponent } from './process-log/process-log.component';
import { MapComponent } from './map/map.component';



@NgModule({
  declarations: [
    AppComponent,
    MainFrameComponent,
    CamerasWidgetComponent,
    CameraBoxComponent,
    GpsComponent,
    ProcessLogComponent,
    MapComponent,

  ],
  imports: [
    BrowserModule,
    HttpClientModule
  ],
  providers: [RosService,
              MarkerService
            ],

  bootstrap: [AppComponent]
})
export class AppModule { }
