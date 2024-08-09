import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';

import { AppComponent } from './app.component';
import { RosService } from './ros.service';
import { CameraComponent } from './components/camera/camera.component';
import { CameraBoxComponent } from './components/camera/camera-box/camera-box.component';
import { AppRoutingModule } from './app-routing.module';
import { PowerPageComponent } from './components/power/power-page/power-page.component';
import { DriveComponent } from './components/drive/drive.component';
import { ArmComponent } from './components/arm/arm.component';
import { SciencePageComponent } from './components/science/science-page/science-page.component';
import { GpsComponent } from './components/gps/gps.component';
import { FormsModule } from '@angular/forms'
import { HttpClientModule } from '@angular/common/http';
import { AntennaComponent } from './components/antenna/antenna.component';
import { GenericComponent } from './components/generic/genric.component';
import { HeaderComponent } from './components/header/header.component';
import { DrivePageComponent } from './pages/drive-page/drive-page.component';
import { ScienceAugerComponent } from './components/science-auger/science-auger.component';

@NgModule({
  declarations: [
    AppComponent,
    AntennaComponent,
    CameraComponent,
    CameraBoxComponent,
    PowerPageComponent,
    DriveComponent,
    ArmComponent,
    SciencePageComponent,
    GpsComponent,
    GenericComponent,
    HeaderComponent,
    DrivePageComponent,
    ScienceAugerComponent,
  ],
  imports: [
    FormsModule,
    BrowserModule,
    HttpClientModule,
    AppRoutingModule,
  ],
  providers: [RosService],
  bootstrap: [AppComponent]
})
export class AppModule { }
