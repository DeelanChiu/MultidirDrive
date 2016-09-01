package org.usfirst.frc.team4253.robot;

import edu.wpi.first.wpilibj.Solenoid;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
	
	Solenoid leftBase;
	Solenoid leftFront;
	Solenoid rightBase;
	Solenoid rightFront;
	
	public Intake (int a, int b, int c, int d) {
		leftBase = new Solenoid(a);
		leftFront = new Solenoid(b);
		rightBase = new Solenoid(c);
		rightFront = new Solenoid(d);
		
		leftBase.set(false);
		leftFront.set(true);
		rightBase.set(false);
		rightFront.set(true);
		
	}
	
	public void letDown (boolean act) {
		if (act) {
			leftBase.set(true);
			rightBase.set(true);
		} else {
			leftBase.set(false);
			rightBase.set(false);			
			
		}
		
	}
	
	public void tuckIn (boolean act) {
		if (act) {
			leftFront.set(false);
			rightFront.set(false);
		} else {
			leftFront.set(true);
			rightFront.set(true);			
			
		}
		
		
	}

}
