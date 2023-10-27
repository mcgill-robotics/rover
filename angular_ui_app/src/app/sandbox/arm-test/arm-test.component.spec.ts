import { ComponentFixture, TestBed } from '@angular/core/testing';

import { ArmTestComponent } from './arm-test.component';

describe('ArmTestComponent', () => {
  let component: ArmTestComponent;
  let fixture: ComponentFixture<ArmTestComponent>;

  beforeEach(() => {
    TestBed.configureTestingModule({
      declarations: [ArmTestComponent]
    });
    fixture = TestBed.createComponent(ArmTestComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
