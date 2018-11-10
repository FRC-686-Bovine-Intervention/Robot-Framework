package org.usfirst.frc.team686.robot.loops;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.*;

import edu.wpi.first.wpilibj.Timer;

import org.usfirst.frc.team686.robot.lib.sensors.GyroBase;
import org.usfirst.frc.team686.robot.lib.sensors.BNO055;
import org.usfirst.frc.team686.robot.lib.sensors.NavX;
import org.usfirst.frc.team686.robot.Constants;
import org.usfirst.frc.team686.robot.command_status.DriveCommand;
import org.usfirst.frc.team686.robot.command_status.DriveState;
import org.usfirst.frc.team686.robot.subsystems.Drive;

/*
 * DriveLoop is the interface between Drive.java and the actual hardware.
 * It runs periodically, taking the commands sent by Drive and sending them to the hardware.
 * In this way, Drive.java does not access the hardware directly.  The benefits of this partition are: 
 * 1) Changes to drive hardware only requires changes to DriveLoop, not Drive
 * 2) DriveLoop can be easily replaced for simulation purposes.
 */

public class DriveLoop implements Loop 
{
	private static DriveLoop instance = new DriveLoop();
	public static DriveLoop getInstance() { return instance; }
	
    private static Drive drive;
	private static GyroBase gyro;
    private DriveState driveState;
    
	public final TalonSRX lMotorMaster;
	public final TalonSRX rMotorMaster;
	public final List<BaseMotorController> lMotorSlaves;
	public final List<BaseMotorController> rMotorSlaves;

	private static final int kVelocityControlSlot = 0;
	private static final int kBaseLockControlSlot = 1;

	private DriveLoop() 
	{
		drive = Drive.getInstance();

		driveState = DriveState.getInstance();
		
		/*****************************************************************
		 * Configure Master Motor Controllers
		 *****************************************************************/
		lMotorMaster = new TalonSRX(Constants.kLeftMotorMasterTalonId);
		rMotorMaster = new TalonSRX(Constants.kRightMotorMasterTalonId);
        
		// Get status at 100Hz (faster than default 50 Hz)
		lMotorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, Constants.kTalonTimeoutMs);
		rMotorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, Constants.kTalonTimeoutMs);

		lMotorMaster.set(ControlMode.PercentOutput, 0.0);
		rMotorMaster.set(ControlMode.PercentOutput, 0.0);
		lMotorMaster.setNeutralMode(NeutralMode.Coast);
		rMotorMaster.setNeutralMode(NeutralMode.Coast);

		lMotorMaster.configOpenloopRamp(Constants.kDriveSecondsFromNeutralToFull, Constants.kTalonTimeoutMs);		
		rMotorMaster.configOpenloopRamp(Constants.kDriveSecondsFromNeutralToFull, Constants.kTalonTimeoutMs);		
		
		// Set up the encoders
		lMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);	// configure for closed-loop PID
		rMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
		lMotorMaster.setSensorPhase(Constants.kLeftMotorSensorPhase);
		rMotorMaster.setSensorPhase(Constants.kRightMotorSensorPhase);
		lMotorMaster.setInverted(Constants.kLeftMotorInverted);
		rMotorMaster.setInverted(Constants.kRightMotorInverted);
		
		// Load velocity control gains
		lMotorMaster.config_kF(kVelocityControlSlot, Constants.kDriveVelocityKf, Constants.kTalonTimeoutMs);
		lMotorMaster.config_kP(kVelocityControlSlot, Constants.kDriveVelocityKp, Constants.kTalonTimeoutMs);
		lMotorMaster.config_kI(kVelocityControlSlot, Constants.kDriveVelocityKi, Constants.kTalonTimeoutMs);
		lMotorMaster.config_kD(kVelocityControlSlot, Constants.kDriveVelocityKd, Constants.kTalonTimeoutMs);
		lMotorMaster.config_IntegralZone(kVelocityControlSlot, Constants.kDriveVelocityIZone, Constants.kTalonTimeoutMs);

		rMotorMaster.config_kF(kVelocityControlSlot, Constants.kDriveVelocityKf, Constants.kTalonTimeoutMs);
		rMotorMaster.config_kP(kVelocityControlSlot, Constants.kDriveVelocityKp, Constants.kTalonTimeoutMs);
		rMotorMaster.config_kI(kVelocityControlSlot, Constants.kDriveVelocityKi, Constants.kTalonTimeoutMs);
		rMotorMaster.config_kD(kVelocityControlSlot, Constants.kDriveVelocityKd, Constants.kTalonTimeoutMs);
		rMotorMaster.config_IntegralZone(kVelocityControlSlot, Constants.kDriveVelocityIZone, Constants.kTalonTimeoutMs);
		
		lMotorMaster.configAllowableClosedloopError(kVelocityControlSlot, Constants.kDriveVelocityAllowableError, Constants.kTalonTimeoutMs);
		rMotorMaster.configAllowableClosedloopError(kVelocityControlSlot, Constants.kDriveVelocityAllowableError, Constants.kTalonTimeoutMs);

		
		// Load base lock control gains
		lMotorMaster.config_kF(kBaseLockControlSlot, Constants.kDriveBaseLockKf, Constants.kTalonTimeoutMs);
		lMotorMaster.config_kP(kBaseLockControlSlot, Constants.kDriveBaseLockKp, Constants.kTalonTimeoutMs);
		lMotorMaster.config_kI(kBaseLockControlSlot, Constants.kDriveBaseLockKi, Constants.kTalonTimeoutMs);
		lMotorMaster.config_kD(kBaseLockControlSlot, Constants.kDriveBaseLockKd, Constants.kTalonTimeoutMs);
		lMotorMaster.config_IntegralZone(kBaseLockControlSlot, Constants.kDriveBaseLockIZone, Constants.kTalonTimeoutMs);

		rMotorMaster.config_kF(kBaseLockControlSlot, Constants.kDriveBaseLockKf, Constants.kTalonTimeoutMs);
		rMotorMaster.config_kP(kBaseLockControlSlot, Constants.kDriveBaseLockKp, Constants.kTalonTimeoutMs);
		rMotorMaster.config_kI(kBaseLockControlSlot, Constants.kDriveBaseLockKi, Constants.kTalonTimeoutMs);
		rMotorMaster.config_kD(kBaseLockControlSlot, Constants.kDriveBaseLockKd, Constants.kTalonTimeoutMs);
		rMotorMaster.config_IntegralZone(kBaseLockControlSlot, Constants.kDriveBaseLockIZone, Constants.kTalonTimeoutMs);

		lMotorMaster.configAllowableClosedloopError(kBaseLockControlSlot, Constants.kDriveBaseLockAllowableError, Constants.kTalonTimeoutMs);
		rMotorMaster.configAllowableClosedloopError(kBaseLockControlSlot, Constants.kDriveBaseLockAllowableError, Constants.kTalonTimeoutMs);
		
		lMotorMaster.configOpenloopRamp(Constants.kDriveVelocityRampRate, 0);
		rMotorMaster.configOpenloopRamp(Constants.kDriveVelocityRampRate, 0);

		/*****************************************************************
		 * Configure Slave Motor Controllers
		 *****************************************************************/
		lMotorSlaves = new ArrayList<BaseMotorController>();	
		rMotorSlaves = new ArrayList<BaseMotorController>();	
		System.out.println("Competition Bot: 1 VictorSPX slave per side");
		lMotorSlaves.add(new VictorSPX(Constants.kLeftMotorSlave1TalonId));
		rMotorSlaves.add(new VictorSPX(Constants.kRightMotorSlave1TalonId));			
		
        for (BaseMotorController lMotorSlave : lMotorSlaves) 
        {
    		lMotorSlave.follow(lMotorMaster);	// give slave the TalonID of it's master
    		lMotorSlave.setNeutralMode(NeutralMode.Coast);
    		lMotorSlave.setInverted(Constants.kLeftMotorInverted);
        }
        for (BaseMotorController rMotorSlave : rMotorSlaves) 
        {
    		rMotorSlave.follow(rMotorMaster);	// give slave the TalonID of it's master
    		rMotorSlave.setNeutralMode(NeutralMode.Coast);
    		rMotorSlave.setInverted(Constants.kRightMotorInverted);
        }
        
		/*****************************************************************
		 * Set Initial Motor Settings
		 *****************************************************************/
		DriveCommand neutralCmd = DriveCommand.COAST();
		setControlMode(neutralCmd);
		setMotors(neutralCmd);
		setNeutralMode(neutralCmd);
		resetEncoders(neutralCmd);        

	
		/*****************************************************************
		 * Select which Gyro is installed
		 *****************************************************************/
		switch (Constants.GyroSelection)
		{
		case BNO055:
			System.out.println("Selected gyro = BNO055");
			gyro = BNO055.getInstance();
			break;
		case NAVX:
		default:
			System.out.println("Selected gyro = NavX");
			gyro = NavX.getInstance();
			break;
		}

	}
	
	
	@Override public void onStart()
	{
		// nothing
	}

	@Override public void onLoop()
	{
		// get status from hardware
		getStatus();
		
		// send new commands to hardware
		sendCommands();
	}

	@Override public void onStop()
	{
		stopMotors();
	}

	private void stopMotors()
	{
		drive.setCommand(DriveCommand.COAST());		// override any incoming commands 
		sendCommands();
	}

	private void getStatus()
	{
		synchronized (driveState)	// lock DriveState until we update it, so that objects reading DriveState don't get partial updates	
		{
			// get Talon control & brake modes (assume right motor is configured identically)
			driveState.setTalonControlMode( lMotorMaster.getControlMode() );
			driveState.setNeutralMode( DriveCommand.getNeutralMode() );
			
			// get encoder values from hardware, set in Drive
			driveState.setLeftDistanceInches(  encoderUnitsToInches( lMotorMaster.getSelectedSensorPosition( Constants.kTalonPidIdx ) ));
			driveState.setRightDistanceInches( encoderUnitsToInches( rMotorMaster.getSelectedSensorPosition( Constants.kTalonPidIdx ) ));
	
			driveState.setLeftSpeedInchesPerSec(  encoderUnitsPerFrameToInchesPerSecond( lMotorMaster.getSelectedSensorVelocity(  Constants.kTalonPidIdx  ) ));
			driveState.setRightSpeedInchesPerSec( encoderUnitsPerFrameToInchesPerSecond( rMotorMaster.getSelectedSensorVelocity(  Constants.kTalonPidIdx  ) ));
				
			/*
			 * measured angle decreases with clockwise rotation
			 * it should increase with clockwise rotation (according to
			 * documentation, and standard right hand rule convention
			 * negate it here to correct
			 */
			driveState.setHeadingDeg( gyro.getHeadingDeg() );
	
			driveState.setMotorCurrent(lMotorMaster.getOutputCurrent(), rMotorMaster.getOutputCurrent() );
			driveState.setMotorPIDError(lMotorMaster.getClosedLoopError( Constants.kTalonPidIdx ), rMotorMaster.getClosedLoopError( Constants.kTalonPidIdx ) );
	
	        switch (driveState.getTalonControlMode())
	        {
	        	case PercentOutput: 
	        		driveState.setMotorStatus(lMotorMaster.getMotorOutputPercent(), rMotorMaster.getMotorOutputPercent() );
	                break;
	
	        	case Position:
	        		driveState.setMotorStatus(lMotorMaster.getSelectedSensorPosition( Constants.kTalonPidIdx ), rMotorMaster.getSelectedSensorPosition( Constants.kTalonPidIdx ) );
	        		break;
	        		
	        	case Velocity:
	        		driveState.setMotorStatus(lMotorMaster.getSelectedSensorVelocity( Constants.kTalonPidIdx ), rMotorMaster.getSelectedSensorVelocity( Constants.kTalonPidIdx ) );
	        		break;
	        		
	        	case Disabled:
	        	default:
	        		driveState.setMotorStatus(lMotorMaster.getMotorOutputPercent(), rMotorMaster.getMotorOutputPercent() );
	        		break;
			}
		}
	}
		
	private void sendCommands()
	{
		DriveCommand newCmd = drive.getCommand();
		
		// Watchdog timer  
		double currentTime = Timer.getFPGATimestamp();
		if (currentTime - newCmd.getCommandTime() > Constants.kDriveWatchdogTimerThreshold)
		{
			// Halt robot if new command hasn't been sent in a while
			stopMotors();
			return;
		}
				
		synchronized(newCmd)	// lock DriveCommand so no one changes it under us while we are sending the commands
		{
			setControlMode(newCmd);
			setMotors(newCmd);
			setNeutralMode(newCmd);
			resetEncoders(newCmd);
		}
	}
	
	
	private void setControlMode(DriveCommand newCmd)
    {
		ControlMode newMode = newCmd.getTalonControlMode();
		
		if (newMode != driveState.getTalonControlMode())
		{
	        switch (newMode)
	        {
	        	case PercentOutput: 
	                break;
	
	        	case Position:
	    			lMotorMaster.selectProfileSlot(kBaseLockControlSlot, Constants.kTalonPidIdx);
	    			rMotorMaster.selectProfileSlot(kBaseLockControlSlot, Constants.kTalonPidIdx);

	        		lMotorMaster.set(ControlMode.Position, (double)(lMotorMaster.getSelectedSensorPosition( Constants.kTalonPidIdx )) );
	        		rMotorMaster.set(ControlMode.Position, (double)(rMotorMaster.getSelectedSensorPosition( Constants.kTalonPidIdx )) );
	        		break;
	        		
	        	case Velocity:
	        		lMotorMaster.selectProfileSlot(kVelocityControlSlot, Constants.kTalonPidIdx);
	        		rMotorMaster.selectProfileSlot(kVelocityControlSlot, Constants.kTalonPidIdx);
	        		break;
	        		
	        	case Disabled:
	        	default:
	        		break;
	        }
		}
	}
	
	
	
	private void setNeutralMode(DriveCommand newCmd)
	{
		NeutralMode newNeutral = DriveCommand.getNeutralMode();
		setNeutralMode(newNeutral);
	}
	
	
	private void setNeutralMode(NeutralMode newNeutral) 
	{
		if (newNeutral != driveState.getNeutralMode()) 
		{
			lMotorMaster.setNeutralMode(newNeutral);
			rMotorMaster.setNeutralMode(newNeutral);
			for (BaseMotorController lMotorSlave : lMotorSlaves)
				lMotorSlave.setNeutralMode(newNeutral);
			for (BaseMotorController rMotorSlave : rMotorSlaves)
				rMotorSlave.setNeutralMode(newNeutral);
		}
	}
	
	
		
	private void setMotors(DriveCommand newCmd)
    {
		double lMotorCtrl = newCmd.getLeftMotor();
		double rMotorCtrl = newCmd.getRightMotor();
		
        switch (newCmd.getTalonControlMode())	// assuming new mode is already configured
        {
        	case PercentOutput:
        		// DriveCommand given in range +/-1, with 1 representing full throttle
        		lMotorMaster.set(ControlMode.PercentOutput, lMotorCtrl);
        		rMotorMaster.set(ControlMode.PercentOutput, rMotorCtrl);
        		break;

        	case Position:
        		// no changes to position set in setControlMode()
        		break;
        		
        	case Velocity:
        		// DriveCommand given in inches/sec
        		// Talon SRX needs RPM in closed-loop mode.
        		// convert inches/sec to encoder edges per 100ms
           		lMotorMaster.set(ControlMode.Velocity, inchesPerSecondToEncoderUnitsPerFrame(lMotorCtrl)); 
        		rMotorMaster.set(ControlMode.Velocity, inchesPerSecondToEncoderUnitsPerFrame(rMotorCtrl));
        		break;
        		
        	case Disabled:
        	default:
        		lMotorMaster.set(ControlMode.Disabled, 0);
        		rMotorMaster.set(ControlMode.Disabled, 0);
        		break;
        }
	}

	// Talon SRX reports position in rotations while in closed-loop Position mode
	private static double encoderUnitsToInches(int _encoderPosition) {	return (double)_encoderPosition / (double)Constants.kQuadEncoderUnitsPerRev  * Constants.kDriveWheelCircumInches; }
	private static int inchesToEncoderUnits(double _inches) { return (int)(_inches / Constants.kDriveWheelCircumInches * Constants.kQuadEncoderUnitsPerRev); }

	// Talon SRX reports speed in RPM while in closed-loop Speed mode
	private static double encoderUnitsPerFrameToInchesPerSecond(int _encoderEdgesPerFrame) { return encoderUnitsToInches(_encoderEdgesPerFrame) / Constants.kQuadEncoderStatusFramePeriod; }
	private static int inchesPerSecondToEncoderUnitsPerFrame(double _inchesPerSecond) { return (int)(inchesToEncoderUnits(_inchesPerSecond) * Constants.kQuadEncoderStatusFramePeriod); }

	
	

	private void resetEncoders(DriveCommand newCmd)
	{
		if (newCmd.getResetEncoders())
		{
			SensorCollection collection = lMotorMaster.getSensorCollection();
			collection.setQuadraturePosition(0, Constants.kTalonTimeoutMs);
			
			collection = rMotorMaster.getSensorCollection();
			collection.setQuadraturePosition(0, Constants.kTalonTimeoutMs);
			
			// cannot reset gyro heading in hardware.  
			// calibration to desired initial pose is done in RobotState.reset() called from Robot.autonomousInit()  
		}
	}	


};
