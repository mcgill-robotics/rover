import { ComponentFixture, TestBed } from '@angular/core/testing';

import { DriveControlComponent } from './drive-control.component';

describe('DrivePageComponent', () => {
  let component: DriveControlComponent;
  let fixture: ComponentFixture<DriveControlComponent>;

  beforeEach(() => {
    TestBed.configureTestingModule({
      declarations: [DriveControlComponent]
    });
    fixture = TestBed.createComponent(DriveControlComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
