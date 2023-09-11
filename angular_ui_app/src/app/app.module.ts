import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';

import { AppComponent } from './app.component';
import { MainFrameComponent } from './main-frame/main-frame.component';
import { RosService } from './ros.service';
import { CamerasWidgetComponent } from './cameras-widget/cameras-widget.component';
import { CameraBoxComponent } from './cameras-widget/camera-box/camera-box.component';


@NgModule({
  declarations: [
    AppComponent,
    MainFrameComponent,
    CamerasWidgetComponent,
    CameraBoxComponent
  ],
  imports: [
    BrowserModule
  ],
  providers: [RosService],
  bootstrap: [AppComponent]
})
export class AppModule { }
