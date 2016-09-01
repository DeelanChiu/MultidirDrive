
package org.usfirst.frc.team4253.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Gyro;



import org.usfirst.frc.team4253.robot.EncoderDrive; //EncoderDrive
import org.usfirst.frc.team4253.robot.Gantry;//import Arm
import org.usfirst.frc.team4253.robot.Intake;
//import org.usfirst.frc.team4253.robot.Vision;//import Vision

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	
	/*Talon rightBack = new Talon(5);
	Talon rightFront = new Talon (4);
	Talon leftFront = new Talon(2);
	Talon leftBack = new Talon (3);*/
	
	Joystick joy = new Joystick(0);
	boolean driveSwitch = true;
	Gyro gyro;

	
	EncoderDrive baseDrive;
	Gantry liftControl;
	Intake in;
	
	//intake
	//Vision camVision;
	//
	

	
	Timer t1;
	
	
    public void robotInit() {
    	baseDrive = new EncoderDrive(0,1,2,3,4, 5);//4 and 5 is sub for strafe port
    	baseDrive.setEncoders(2, 0, 4); //4 is sub for strafe encod port
    	
    	liftControl = new Gantry(6, 7); // 6 and 7 is sub for arm port
    	liftControl.setEncoders(6); //6 is sub for arm encod port
    	
    	//testRelay = new Relay (1);
    	
    	t1 = new Timer();
    	t1.start();
    	
    	gyro = new Gyro (0);
    	gyro.reset();
    	
    	
    	//camVision = new Vision();
    	
    	LiveWindow.setEnabled(true);
    	
    	
    	/*LiveWindow.addSensor("BaseSensors", "encodLeft", baseDrive.encodLeft);
    	LiveWindow.addSensor("BaseSensors", "encodRight", baseDrive.encodRight);*/
    	
    
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	baseDrive.straightDist(1000, 0.8, 0, 0);
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	//baseDrive.getEncodValues();
    	/*SmartDashboard.putNumber("encodLeft",baseDrive.encodLeft.get());
    	SmartDashboard.putNumber("encodRight",baseDrive.encodRight.get());
    	
    	SmartDashboard.putNumber("joyLeft",joy.getRawAxis(1));
    	SmartDashboard.putNumber("joyRight",joy.getRawAxis(3));
    	
    	SmartDashboard.putBoolean("driveSwitch", driveSwitch);
    	SmartDashboard.putNumber("timer", t1.get()); */
    	
    	SmartDashboard.putNumber("accel", baseDrive.accel.getX()); 
    	
    	SmartDashboard.putNumber("gyro", gyro.getAngle()); 
    	
    	if (joy.getRawButton(10) && t1.get()>0.5) {
    		if (driveSwitch) driveSwitch = false;
    		else driveSwitch = true;
    		
    		t1.reset();
    	}
    	
    	
    	//baseDrive.standardTank(joy.getRawAxis(3), joy.getRawAxis(1), joy.getRawAxis(2));
    	
    	baseDrive.FOD(joy.getRawAxis(3)/*rightY*/, joy.getRawAxis(2)/*rightX*/, joy.getRawAxis(0)/*turn*/, gyro.getAngle(), true);
    	//baseDrive.pJoyTank(joy.getRawAxis(0),joy.getRawAxis(3), joy.getRawAxis(2), !driveSwitch);
    	
    	
    	
    	if (joy.getRawButton(6) /*&& liftControl.armEncod.get()<1*/) //1 is sub for max
			liftControl.armTmp(1.0);
		else if (joy.getRawButton(8) /*&& liftControl.armEncod.get()>0*/)  //0 is sub for min
			liftControl.armTmp(-1.0);
		else {
			liftControl.armTmp(0.0);
			//liftControl.pPwr=0;
			//liftControl.currPwr=0;
		}
    	 /*
    	if (joy.getRawButton(4)){
    		in.letDown(true);
    	} else if (joy.getRawButton(3)){
    		in.letDown(false);
    		
    	}
    	
    	if (joy.getRawButton(1)){
    		in.tuckIn(false);
    	} else if (joy.getRawButton(2)){
    		in.tuckIn(true);
    		
    	}
    	*/
    	
    	//camVision.view();
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	
    	
    	baseDrive.getEncodValues();
    	
    	SmartDashboard.putNumber("encodLeft",baseDrive.encodLeft.get());
    	SmartDashboard.putNumber("encodRight",baseDrive.encodRight.get());   
    	
    	LiveWindow.addSensor("BaseSensors", 0, baseDrive.encodLeft);
    	LiveWindow.addSensor("BaseSensors", 2, baseDrive.encodRight);
    	
    	
    }
    

    
}
