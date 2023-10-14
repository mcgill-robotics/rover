import { ComponentFixture, TestBed } from '@angular/core/testing';

import { CameraTestComponent } from './camera-test.component';

describe('CameraTestComponent', () => {
  let component: CameraTestComponent;
  let fixture: ComponentFixture<CameraTestComponent>;

  beforeEach(() => {
    TestBed.configureTestingModule({
      declarations: [CameraTestComponent]
    });
    fixture = TestBed.createComponent(CameraTestComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
