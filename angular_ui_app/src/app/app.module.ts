import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';

import { AppComponent } from './app.component';
import { MainFrameComponent } from './main-frame/main-frame.component';
import { RosService } from './ros.service';
import { CamerasWidgetComponent } from './cameras-widget/cameras-widget.component';
import { CameraBoxComponent } from './cameras-widget/camera-box/camera-box.component';
import { TurtlesimComponent } from './sandbox/turtlesim/turtlesim.component';
import { AppRoutingModule } from './app-routing.module';
import { PowerPageComponent } from './components/power/power-page/power-page.component';
import { DrivePageComponent } from './components/drive/drive-page/drive-page.component';
import { ArmPageComponent } from './components/arm/arm-page/arm-page.component';
import { SciencePageComponent } from './components/science/science-page/science-page.component';
import { GpsPageComponent } from './components/gps/gps-page/gps-page.component';
import { Button1Component } from './sandbox/buttons/button1/button1.component';
import { CameraTestComponent } from './sandbox/camera-test/camera-test.component';


@NgModule({
  declarations: [
    AppComponent,
    MainFrameComponent,
    CamerasWidgetComponent,
    CameraBoxComponent,
    TurtlesimComponent,
    PowerPageComponent,
    DrivePageComponent,
    ArmPageComponent,
    SciencePageComponent,
    GpsPageComponent,
    Button1Component,
    CameraTestComponent
  ],
  imports: [
    BrowserModule,
    AppRoutingModule
  ],
  providers: [RosService],
  bootstrap: [AppComponent]
})
export class AppModule { }
