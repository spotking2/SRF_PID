package org.usfirst.frc.team3826.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SRF_PID { //v1.1.1
	/*	Fixed instance overwrite problem
	 *  Untested
	 */
	
	private int P = 0, I = 1, D = 2;
	private double[] k = new double[3];
	private double errorSum = 0, lastError = 0;
	private double setpoint;
	private boolean reversed = false;
	private double max = 1, min = -1;
	private double lastTime = 0;
	private double[] initialK = new double[3];
	private double[] mult = new double[] {1,1,1};
	
	//Tuner
	private Joystick xbox;
	int kValue = 0;
	int adjustP = 0;
	int adjustI = 0;
	int adjustD = 0;
	boolean adjusted = false;
	int[] multPID = new int[] {adjustP, adjustI, adjustD};
	
	boolean letUp1 = true;						//A
	boolean letUp2 = true;						//B
	boolean letUp3 = true;						//X
	boolean letUp4 = true;						//Y
	boolean letUp7 = true;						//Back
	
	String kValueName = "P";
	
	public SRF_PID(Joystick trollPID) {
		xbox = trollPID;
	}
	
	public void setLimits(double high, double low)
	{
		max = high;
		min = low;
	}
	
	public void setReverse(boolean reverse)
	{
		reversed = reverse;
	}
	
	public void setPID(double nP, double nI, double nD, boolean init)
	{
		k[P] = nP;
		k[I] = nI;
		k[D] = nD;
		
		if(init){
			initialK[P] = nP;
			initialK[I] = nI;
			initialK[D] = nD;
		}
	}
	
	public void adjustPID(double adjustP, double adjustI, double adjustD, boolean init)
	{
		k[P]+=adjustP;
		k[I]+=adjustI;
		k[D]+=adjustD;
		
		if(init){
			initialK[P]+=adjustP;
			initialK[I]+=adjustI;
			initialK[D]+=adjustD;
		}
	}
	
	public void setSetpoint(double target)
	{
		setpoint = target;
	}
	
	public double computePID(double current, double timeNow)
	{
		double output;
		double error;
		double dT = timeNow - lastTime;
		lastTime = timeNow;
		
		if(timeNow == 0)
			errorSum = 0;
		
		error = setpoint - current;
		errorSum+=error;
		//System.out.println("("+kP+"*"+error+")+("+kI+"*"+dT+"*"+errorSum+")+"+kD+"/"+dT+"*("+error+"-"+lastError+")");
		if(dT != 0)
			output = (k[P] * error) + (k[I] * dT * errorSum) + k[D] / dT * (error - lastError); //Mo is typically not relevant in current output computation
		else
			output = (k[P] * error) + (k[I] * dT * errorSum);
		
		
		if(reversed)
			output*=-1;
		
		if(output > max)
			output = max;
		else if(output < min)
			output = min;
		//System.out.println("="+output);
		lastError = error; //Go here for math basis
							//https://library.automationdirect.com/methods-behind-pid-loop-control/
		return output;
	}
	
	public void setMult(double nP, double nI, double nD)
	{
		mult[P] = nP;
		mult[I] = nI;
		mult[D] = nD;
	}
	
	public void adjustMult(double adjustP, double adjustI, double adjustD)
	{
		mult[P]+=adjustP;
		mult[I]+=adjustI;
		mult[D]+=adjustD;
	}

	public void run() {
		if(xbox.getRawButton(1) && letUp1) 		//A
		{
			multPID[kValue] -= 1;
			letUp1 = false;	
		} 
		else if(xbox.getRawButton(2) && letUp2) 	//B
		{
			multPID[kValue] += 1;
			letUp2 = false;
		}
		else if(xbox.getRawButton(3) && letUp3) 	//X
		{
			kValue++;
			
			if(kValue==0)
				kValueName = "P";
			else if(kValue==1)
				kValueName = "I";
			else if(kValue==2)
				kValueName = "D";
			else {
				kValue = 0;
				kValueName = "P";
			}
			letUp3 = false;
			
		}
		else if(xbox.getRawButton(4) && letUp4)		//Y
		{
			multPID[kValue] += .1;
			letUp4 = false;
		}
		else if(xbox.getRawButton(7) && letUp7)		//Back
		{
			multPID[kValue] -= .1;
		}
	
		if(adjusted){
			adjustMult(adjustP, adjustI, adjustD);
			adjustP = 0;
			adjustI = 0;
			adjustD = 0;
		}
		
		if(!xbox.getRawButton(1))			//A
			letUp1 = true;
		if(!xbox.getRawButton(2))			//B
			letUp2 = true;
		if(!xbox.getRawButton(3))			//X
			letUp3 = true;
		if(!xbox.getRawButton(4))			//Y
			letUp4 = true;
		if(!xbox.getRawButton(7))			//Back
			letUp7 = true;
		
		SmartDashboard.putNumber("P value", k[1]);
		SmartDashboard.putNumber("I value", k[2]);
		SmartDashboard.putNumber("D value", k[3]);
		SmartDashboard.putString("Value Being Changed", kValueName);
	}
	
}
