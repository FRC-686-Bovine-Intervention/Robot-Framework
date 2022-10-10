package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class TestDriveStatus extends StatusBase{
    private static TestDriveStatus instance = null;
    public static TestDriveStatus getInstance() {if(instance == null){instance = new TestDriveStatus();}return instance;}

    private ControlMode talonControlMode = ControlMode.Disabled;
	private NeutralMode neutralMode;
	
	private double lDistanceInches, rDistanceInches;
	private double lSpeedInchesPerSec, rSpeedInchesPerSec;
	private double heading;
	
	private double lMotorCurrent, rMotorCurrent;
	private double lMotorStatus, rMotorStatus;
	private double lMotorPIDError, rMotorPIDError;
	
	public TestDriveStatus() {}
	
	public synchronized void setTalonControlMode(ControlMode val) { talonControlMode = val; }
	public synchronized ControlMode getTalonControlMode() { return talonControlMode; }
	
	public synchronized void setNeutralMode(NeutralMode val) { neutralMode = val; }
	public synchronized NeutralMode getNeutralMode() { return neutralMode; }
	
	public synchronized void setLeftDistanceInches(double val)  { lDistanceInches = val; }
	public synchronized void setRightDistanceInches(double val) { rDistanceInches = val; }

	public synchronized double getLeftDistanceInches()  { return lDistanceInches; }
	public synchronized double getRightDistanceInches() { return rDistanceInches; }

	public synchronized void setLeftSpeedInchesPerSec(double val)  { lSpeedInchesPerSec = val; }
	public synchronized void setRightSpeedInchesPerSec(double val) { rSpeedInchesPerSec = val; }
	
	public synchronized double getLeftSpeedInchesPerSec()  { return lSpeedInchesPerSec; }
	public synchronized double getRightSpeedInchesPerSec() { return rSpeedInchesPerSec; }

	public synchronized void setMotorCurrent(double lVal, double rVal) { lMotorCurrent = lVal; rMotorCurrent = rVal; }
	public synchronized void setMotorStatus(double lVal, double rVal) { lMotorStatus = lVal; rMotorStatus = rVal; }				// current settings, read back from Talon (may be different than commanded values)
	public synchronized void setMotorPIDError(double lVal, double rVal) { lMotorPIDError = lVal; rMotorPIDError = rVal; }
    
	public synchronized double getLeftMotorCurrent()  { return lMotorCurrent; }
	public synchronized double getRightMotorCurrent() { return rMotorCurrent; }

	public synchronized double getLeftMotorCtrl()  { return lMotorStatus; }
	public synchronized double getRightMotorCtrl() { return rMotorStatus; }

	public synchronized double getLeftMotorPIDError()  { return lMotorPIDError; }
	public synchronized double getRightMotorPIDError() { return rMotorPIDError; }

	public synchronized void setHeadingDeg(double val) { setHeading(val*Math.PI/180.0); }
    public synchronized void setHeading(double val) { heading = val; }

    public synchronized double getHeading() { return heading; };
    public synchronized double getHeadingDeg() { return heading*180.0/Math.PI; }
}
