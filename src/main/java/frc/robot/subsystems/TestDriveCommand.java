package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.util.Kinematics.WheelSpeed;

public class TestDriveCommand {
    public enum DriveControlMode { OPEN_LOOP, BASE_LOCKED, POSITION_SETPOINT, VELOCITY_SETPOINT, VELOCITY_HEADING, TURN_TO_HEADING }

	// all member variables should be private to force other object to use the set/get access methods
	// which are synchronized to allow multi-thread synchronization	
	private DriveControlMode driveMode = DriveControlMode.OPEN_LOOP;
	private ControlMode talonMode = ControlMode.PercentOutput;
	private WheelSpeed wheelSpeed = new WheelSpeed();
	private static NeutralMode neutralMode;	// Brake or Coast
	private boolean resetEncoders;
    private double commandTime;
    
    public TestDriveCommand(double _left, double _right)
    {
        this(DriveControlMode.OPEN_LOOP, _left, _right, NeutralMode.Coast);
    }

    public TestDriveCommand(double _left, double _right, NeutralMode _neutralMode)
    {
        this(DriveControlMode.OPEN_LOOP, _left, _right, _neutralMode);
    }

    public TestDriveCommand(DriveControlMode _mode, double _left, double _right, NeutralMode _neutralMode) 
    {
    	setDriveMode(_mode);
    	setMotors(_left, _right);
    	setNeutralMode(_neutralMode);
    	resetEncoders = false;
    }

    public TestDriveCommand(DriveControlMode _mode, WheelSpeed _vWheel, NeutralMode _neutralMode) 
    {
    	setDriveMode(_mode);
    	setMotors(_vWheel);
    	setNeutralMode(_neutralMode);
    	resetEncoders = false;
    }

    public void scale(double _factor)
    {
    	wheelSpeed.scale(_factor);
    }
    
    
    public synchronized void setDriveMode(DriveControlMode _driveMode) 
    {
    	driveMode = _driveMode;
    	switch (driveMode)
    	{
    	case OPEN_LOOP:
    		talonMode = ControlMode.PercentOutput;
    		break;
    		
    	case BASE_LOCKED:
            talonMode = ControlMode.Position;
        
        case POSITION_SETPOINT:
        case TURN_TO_HEADING:
            talonMode = ControlMode.MotionMagic;
            break;
    		
    	case VELOCITY_SETPOINT:
    	case VELOCITY_HEADING:
     		talonMode = ControlMode.Velocity;
    		break;
    		
    	default:
    		talonMode = ControlMode.Disabled;
    		break;
    	}
    }
    public synchronized DriveControlMode getDriveControlMode() { return driveMode; }
    public synchronized ControlMode getTalonControlMode() { return talonMode; }
    
    public synchronized void   setMotors(WheelSpeed _wheelSpeed) { wheelSpeed.left = _wheelSpeed.left; wheelSpeed.right = _wheelSpeed.right; setCommandTime(); }
    public synchronized void   setMotors(double _left, double _right) { wheelSpeed.left = _left; wheelSpeed.right = _right; setCommandTime(); }
    public synchronized double getLeftMotor()  { return wheelSpeed.left; }
    public synchronized double getRightMotor() { return wheelSpeed.right; }
    public synchronized double getSpeed() { return (wheelSpeed.left + wheelSpeed.right)/2.0; }

    public synchronized void        setNeutralMode(NeutralMode _neutralMode) { neutralMode = _neutralMode; }
    public synchronized static NeutralMode getNeutralMode()  { return neutralMode; }
    
    public synchronized void    setResetEncoders() { resetEncoders = true; }
    public synchronized boolean getResetEncoders() 
    {	
    	// self-clearing reset on read
    	boolean rv = resetEncoders; 
    	resetEncoders = false; 
    	return rv; 
    }	
    
    public synchronized void   setCommandTime() { commandTime = Timer.getFPGATimestamp(); }
    public synchronized double getCommandTime() { return commandTime; } 
    
    
    // special constant commands
    public static TestDriveCommand COAST() { return new TestDriveCommand(DriveControlMode.OPEN_LOOP, 0, 0, NeutralMode.Coast); }
    public static TestDriveCommand BRAKE() { return new TestDriveCommand(DriveControlMode.OPEN_LOOP, 0, 0, NeutralMode.Brake); }
    
    
    @Override
    public synchronized String toString() 
    {
    	return String.format("%s, %s, %s, L/R: (%+7.3f, % 7.3f)", driveMode, talonMode, neutralMode, wheelSpeed.left, wheelSpeed.right);
    }
}
