package org.usfirst.frc.team3826.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SRF_PID { //v1.1.1
	/*	Fixed instance overwrite problem
	 *  Untested
	 * 
	 * 
	 * 
	 */
	
	private int P = 0, I = 1, D = 2;
	private double[] k = new double[3];
	private double errorSum = 0, lastError = 0;
	private double setpoint;
	private boolean reversed = false;
	private double max = 1, min = -1;
	private double lastTime = 0;
	private Joystick xbox;

	private double[] initialK = new double[3];
	private double[] mult = new double[] {1,1,1};
	
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
	
	public void smartDashPrint() {
		SmartDashboard.putNumber("P value", k[1]);
		SmartDashboard.putNumber("I value", k[2]);
		SmartDashboard.putNumber("D value", k[3]);
	}
	
	public void run() {
		
	}
	
}
