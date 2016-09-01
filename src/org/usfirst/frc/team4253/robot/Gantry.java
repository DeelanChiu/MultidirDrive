package org.usfirst.frc.team4253.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.AnalogTrigger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Gantry {
	
	Talon armMotorL;
	Talon armMotorR;
	
	Encoder armEncod;
	
	//AnalogTrigger armLimit;
	
	public Gantry (int a, int b) {
		armMotorL = new Talon(a);
		armMotorR = new Talon(b);
		
		armMotorL.setSafetyEnabled(true);
		armMotorL.setExpiration(.1);
		
		armMotorR.setSafetyEnabled(true);
		armMotorR.setExpiration(.1);
		
		//armLimit = new AnalogTrigger(armEncod);
		
	}
	
	public void setEncoders (int a) {
		armEncod = new Encoder(a,a+1, true, Encoder.EncodingType.k4X);
		
		/*armLimit = new AnalogTrigger(a);
		armLimit.setLimitsRaw(0,1); //0 sub for min, 1 sub for max*/
		
	}
	
	private double pPwr = 0;
	final private double num = Math.pow(1.0,-25.0);
	final private double fullPwr= 1.0;
	private double currPwr = 0;
	
	/* public void TMPup () { //if up
		
		if (armLimit.getInWindow()) {
			
			currPwr = pPwr; // ref past power
			
			if (currPwr<fullPwr)
				currPwr += num/(fullPwr-pPwr); //increment
				
			armMotor.set(currPwr);
			
			pPwr = currPwr; //save current power
		
		}
	}
	
	public void TMPdown () { //if down
		
		if (armLimit.getInWindow()) {
		
		//if (pPwr>fullPwr)
			//pPwr -= num/(pPwr-fullPwr);
			
			armMotor.set(pPwr);
			
			currPwr = pPwr; // ref past power
			
			if (currPwr>-fullPwr)
				currPwr -= num/(fullPwr-pPwr); //increment
				
			armMotor.set(currPwr);
			
			pPwr = currPwr; //save current power
		
		}
		
	} */
	
	public void armTmp (double targetPwr) { //smooth drive, but less sharp movement
		
		currPwr = pPwr;
		
		if (currPwr<targetPwr)
			currPwr += num/(fullPwr-currPwr);
		if (currPwr>targetPwr)
			currPwr -= num/(currPwr-targetPwr);
		
		armMotorL.set(currPwr);
		armMotorR.set(-currPwr);
		
		pPwr = currPwr;
		
	}
	
	//controller of both methods... nvm
	
	public void reachLevel (int level) {
		while (Math.abs(armEncod.get()-level)>5) { //5 is sub for error
		if (armEncod.get()<level)
			armTmp(1.0);
			//TMPup();
		else if (armEncod.get()>level)
			armTmp(-1.0);
			//TMPdown();
		}
		
		armTmp(0);
		//pPwr=0;
		
	}
	
	
	//if neither is on, have to reset variables

}