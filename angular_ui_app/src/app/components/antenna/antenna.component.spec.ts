import { ComponentFixture, TestBed } from '@angular/core/testing';

import { AntennaComponent } from './antenna.component';

describe('AntennaComponent', () => {
  let component: AntennaComponent;
  let fixture: ComponentFixture<AntennaComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [AntennaComponent]
    })
    .compileComponents();
    
    fixture = TestBed.createComponent(AntennaComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
