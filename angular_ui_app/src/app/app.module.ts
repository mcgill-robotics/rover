import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';

import { AppComponent } from './app.component';
import { RosService } from './ros.service';
import { CameraComponent } from './components/camera/camera.component';
import { CameraBoxComponent } from './components/camera/camera-box/camera-box.component';
import { AppRoutingModule } from './app-routing.module';
import { PowerPageComponent } from './components/power/power-page/power-page.component';
import { ArmComponent } from './components/arm/arm.component';
import { SciencePageComponent } from './components/science/science-page/science-page.component';
import { GpsPageComponent } from './components/gps/gps-page/gps-page.component';
import { Button1Component } from './sandbox/buttons/button1/button1.component';
import { CameraTestComponent } from './sandbox/camera-test/camera-test.component';
import { ArmTestComponent } from './sandbox/arm-test/arm-test.component';

import { FormsModule } from '@angular/forms'
import { AntennaComponent } from './components/antenna/antenna.component';
import { GenericComponent } from './components/generic/genric.component';
import { HeaderComponent } from './components/header/header.component';
import { DrivePageComponent } from './pages/drive-page/drive-page.component';
import { NavbarComponent } from './components/navbar/navbar.component';
import { TestPageComponent } from './pages/test-page/test-page.component';
import { ODriveComponent } from './components/o-drive/o-drive.component';

@NgModule({
  declarations: [
    AppComponent,
    AntennaComponent,
    CameraComponent,
    CameraBoxComponent,
    PowerPageComponent,
    ArmComponent,
    SciencePageComponent,
    GpsPageComponent,
    Button1Component,
    CameraTestComponent,
    ArmTestComponent,
    GenericComponent,
    HeaderComponent,
    DrivePageComponent,
    NavbarComponent,
    TestPageComponent,
    ODriveComponent
  ],
  imports: [
    FormsModule,
    BrowserModule,
    AppRoutingModule,
  ],
  providers: [RosService],
  bootstrap: [AppComponent]
})
export class AppModule { }
