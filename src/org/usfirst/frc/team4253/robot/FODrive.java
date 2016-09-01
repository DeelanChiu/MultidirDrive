package org.usfirst.frc.team4253.robot;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



//import org.usfirst.frc.team4253.robot.EncoderDrive;

public class EncoderDrive {
	
	Encoder encodLeft;
	Encoder encodRight;
	Encoder encodStrafe;
	
	Accelerometer accel;
	
	final private RobotDrive baseDrive;
	
	Talon strafe1;
	Talon strafe2;
	
	//make RobotDrive project
	
	public EncoderDrive (int a, int b, int c, int d, int e, int f) {
		Talon rightBack = new Talon(a);
		Talon rightFront = new Talon (b);
		Talon leftBack = new Talon (c);
		Talon leftFront = new Talon(d);
		
		rightBack.setSafetyEnabled(true); rightFront.setSafetyEnabled(true);
		leftBack.setSafetyEnabled(true); leftFront.setSafetyEnabled(true);
		
		double exp = 0.1;
		rightBack.setExpiration(exp); rightFront.setExpiration(exp);
		leftBack.setExpiration(exp); leftFront.setExpiration(exp);
		
		
		baseDrive = new RobotDrive(rightBack, rightFront, leftBack, leftFront);
		//baseDrive = new RobotDrive(a, b, c, d);
		strafe1 = new Talon(e);
		strafe2 = new Talon(f);
		
		
		strafe1.setSafetyEnabled(true); strafe1.setExpiration(exp);
		strafe2.setSafetyEnabled(true); strafe2.setExpiration(exp);
		
		accel = new BuiltInAccelerometer();
		
		
		
	}
	
	public void setEncoders (int left1, int right1, int strafe1) {
		//right encoder type?
		encodLeft = new Encoder(left1,left1+1, true, Encoder.EncodingType.k4X);
		encodRight = new Encoder(right1, right1+1, true, Encoder.EncodingType.k4X);
		encodStrafe =  new Encoder(strafe1, strafe1+1, true, Encoder.EncodingType.k4X);
		
		
		/*encodLeft.startLiveWindowMode();
		encodRight.startLiveWindowMode();*/
		
    	/*LiveWindow.addSensor("BaseSensors", "encodLeft", encodLeft);
    	LiveWindow.addSensor("BaseSensors", "encodRight", encodRight);*/
		
		
	}
		

	
	public void strafing (double strafe) {
		strafe1.set(strafe);
		strafe2.set(strafe);
		
	}
	
	public void standardTank (double right, double left, double strafe){
		baseDrive.tankDrive(right, left);
		strafing(strafe);
		
	}
	
	
	
	double pRight = 0;
	double pLeft = 0;
	double pTankStrafe = 0;
	
	
	final private double num = Math.pow(10, -20.0); //10 0s
	
	public double [][] MatrixMult (double[][] A, double[][] B) {
		double [][] output = new double [1][1];
		
		output[0][0]= (A[0][0]*B[0][0]) + (A[0][1]*B[1][0]);
		output[0][1]= (A[0][0]*B[0][1]) + (A[0][1]*B[1][1]);
		output[1][0]= (A[1][0]*B[0][0]) + (A[1][1]*B[1][0]);
		output[1][1]= (A[1][0]*B[0][1]) + (A[1][1]*B[1][1]);
		
		return output;		
	}
		
	public double[] axisCalc (double joyX, double joyY, double gyro) { //returns x and y vectors of turned robot
		double output [] = new double [1];
		
		double [][] joyMatrix =  { {joyY, 0}, {0, joyX} };
		double [][] gyroMatrix = { {Math.cos(gyro), -Math.sin(gyro)}, {Math.sin(gyro), Math.cos(gyro)} };
		
		output[0] = MatrixMult(joyMatrix, gyroMatrix)[0][1]; //x
		output[1] = MatrixMult(joyMatrix, gyroMatrix)[1][0]; //y
		
		return output;
	}
	
	
	public void pStandardTank (double right, double left, double strafe){

	
		if (right>pRight) pRight+= num/(right-pRight) ;
		if (right<pRight) pRight-= num/(pRight-right);
		
		if (left>pLeft) pLeft+= num/(left-pLeft);
		if (left<pLeft) pLeft-= num/(pLeft-left);

		if (strafe>pTankStrafe) pTankStrafe += num/(strafe-pTankStrafe);
		if (strafe<pTankStrafe) pTankStrafe -= num/(pTankStrafe-strafe);
		
		baseDrive.tankDrive(pRight, pLeft);
		strafe1.set(strafe);
		strafe2.set(strafe);
		
		pRight=right;
		pLeft=left;
		pTankStrafe = strafe;
			
	}
	
	double M1L = 0;
	double M1R = 0;
	boolean collect = false;
	
	double pGyro = 0;
	double totGyro = 0;
	double pTotGyro = 0;
	
	public void FOD (double Xax, double Yax, double turn, double gyro, boolean activate) {
		turn = thresh(turn);
		
		//gyroVal filter
		if (turn>0){
			if (!collect) {
				collect=true;
				pGyro = gyro;	
				pTotGyro = totGyro;
			}
			
			totGyro /*variable*/= pTotGyro /*static*/ + (gyro /*variable*/ - pGyro /*static*/);
			
		} else {
			collect = false;
			
		}
		
		double [] axis = axisCalc(Xax, Yax, totGyro); //x, y
		
		M1L = (axis[1]+turn)/(1+turn);
		M1R = (axis[1]-turn)/(1+turn);
		
		pStandardTank(M1R, M1L, axis[0]);
		
		
	}
	
	public void joyTank (double turn, double straight, double strafe) {
		
		if (turn>0.2) baseDrive.tankDrive(turn, -turn);
		else {
			baseDrive.tankDrive(straight, straight);
			strafing(strafe);
		}
	}
	
	double pTurn = 0;
	double pStraight = 0;
	double pJoyStrafe = 0;
	final private double pNum = Math.pow(10.0,-20.0); //10 0s
	
	public void pJoyTank (double turn, double straight, double strafeVal, boolean activate) {
		if (activate) {
			turn = thresh(turn); straight = thresh(straight); strafeVal = thresh(strafeVal);
			
			
			if (straight>pStraight) pStraight += pNum/(straight-pStraight);
			if (straight<pStraight) pStraight -= pNum/(pStraight-straight);
			
			if (turn>pTurn) pTurn+= pNum/(turn-pTurn) ;
			if (turn<pTurn) pTurn-= pNum/(pTurn-turn);
			
			if (strafeVal>pJoyStrafe) pJoyStrafe += pNum/(strafeVal-pJoyStrafe);
			if (strafeVal<pJoyStrafe) pJoyStrafe -= pNum/(pJoyStrafe-strafeVal);
			
			if (turn>0) baseDrive.tankDrive(pTurn, -pTurn);
			else {
				baseDrive.tankDrive(pStraight, pStraight);
				strafing(pJoyStrafe);
			}
			
			pTurn=turn;
			pStraight=straight;
			pJoyStrafe=strafeVal;
		}
	}
	
	
	public int getEncodValues () {
		SmartDashboard.putNumber("encodLeft",encodLeft.get());
		SmartDashboard.putNumber("encodRight",encodRight.get());
		
		return (encodLeft.get()+encodRight.get())/2;
	
	}
	
	public void resetEncod () {
		encodLeft.reset();
		encodRight.reset();
	}
	
	public void straightDist (double straightDist, double straightSpeed, double sideDist, double sideSpeed) { 
    	//dist is abs value, speed is dir
    	resetEncod();
    	double left=0.0;
    	double right=0.0;
    	double strafe = 0.0;
    			
    	double corr = 0.1;
    	
    	while ( Math.abs( encodLeft.get() ) < straightDist && err(straightDist, encodLeft.get(), 5 )
    			&& Math.abs(encodRight.get()) < straightDist && err(straightDist, encodRight.get(), 5)
    			&& Math.abs(encodStrafe.get()) < sideDist && err(sideDist, encodStrafe.get(), 5) ) {    	
    		
    		if (Math.abs(encodLeft.get() ) < Math.abs(straightDist) && err(straightDist, encodLeft.get(), 5)) {
    			//left didn't reach target yet
    			if (encodLeft.get()>encodRight.get() && err(encodLeft.get(), encodRight.get(), 2) ) {
    				//auto-correct
    				if (straightSpeed >= 0) left=straightSpeed-corr; //if u want to go forward, slow left down
    				else left=straightSpeed+corr; //if u want to go back, slow left down
    				
    			} else left = straightSpeed; 
    		} //if reached target, left is 0 by default
    		
    		if (Math.abs(encodRight.get() ) < Math.abs(straightDist) && err(straightDist, encodRight.get(), 5)) {
    			if (encodRight.get()>encodLeft.get() && err(encodLeft.get(), encodRight.get(), 2) ) {
    				if (straightSpeed >= 0) right=straightSpeed-corr;
    				else right=straightSpeed+corr;
    				
    			} else right = straightSpeed;
    		}
    		
    		if (Math.abs(encodStrafe.get() ) < Math.abs(sideDist) && err(sideDist, encodStrafe.get(), 5)) {
    			strafe = sideSpeed;
    		}
    		
    		standardTank(right, left, strafe);
    	}
    	
    	standardTank(0,0,0);
    	
    }
    
    public boolean err (double num1, double num2, double error){
    	
    	if (Math.abs(num1-num2)<error) //insignificant error, its k
    		return false;
    	return true;
    	
    }
	
    public double thresh (double num){
    	double error = 0.02;
    	if (Math.abs(num)<error) //insignificant read, no feed
    		return 0;
    	else return num;
    	
    }
	
	
}
