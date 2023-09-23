import { NgModule } from '@angular/core';
import { CommonModule } from '@angular/common';
import {RouterModule, Routes} from '@angular/router'
import { TurtlesimComponent } from './sandbox/turtlesim/turtlesim.component';


const routes: Routes = [
  {path: 'turtlesim', component: TurtlesimComponent}
]


@NgModule({
  declarations: [],
  imports: [RouterModule.forRoot(routes),
  ],
  exports: [RouterModule]
})
export class AppRoutingModule { }
