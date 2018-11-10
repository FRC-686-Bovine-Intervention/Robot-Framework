package org.usfirst.frc.team686.robot;

import org.usfirst.frc.team686.robot.auto.*;
import org.usfirst.frc.team686.robot.command_status.DriveState;
import org.usfirst.frc.team686.robot.command_status.RobotState;
import org.usfirst.frc.team686.robot.lib.joystick.ArcadeDriveJoystick;
import org.usfirst.frc.team686.robot.lib.joystick.JoystickControlsBase;
import org.usfirst.frc.team686.robot.subsystems.Drive;
import org.usfirst.frc.team686.robot.subsystems.Superstructure;
import org.usfirst.frc.team686.robot.lib.util.DataLogger;
import org.usfirst.frc.team686.robot.command_status.DriveCommand;

import java.util.TimeZone;

import org.usfirst.frc.team686.robot.lib.util.CrashTracker;
import org.usfirst.frc.team686.robot.lib.util.DataLogController;
import org.usfirst.frc.team686.robot.lib.util.Pose;
import org.usfirst.frc.team686.robot.loops.DriveLoop;
import org.usfirst.frc.team686.robot.loops.LoopController;
import org.usfirst.frc.team686.robot.loops.RobotStateLoop;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;


public class Robot extends IterativeRobot {
	
	PowerDistributionPanel pdp = new PowerDistributionPanel();
	
	JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();

	RobotState robotState = RobotState.getInstance();
	Drive drive = Drive.getInstance();
	Superstructure superStructure = Superstructure.getInstance();
	
	AutoModeExecuter autoModeExecuter = null;
	
	LoopController loopController;
	
	SmartDashboardInteractions smartDashboardInteractions;
	DataLogController robotLogger;

	UsbCamera usbCamera;

	
	enum OperationalMode 
    {
    	DISABLED(0), AUTONOMOUS(1), TELEOP(2), TEST(3);
    	
    	private int val;
    	
    	private OperationalMode (int val) {this.val = val;}
    	public int getVal() {return val;}
    } 
    
    OperationalMode operationalMode = OperationalMode.DISABLED;
    
    public Robot() {
    	CrashTracker.logRobotConstruction();
    }
    
    
	@Override
	public void robotInit() {
		try
    	{
    		CrashTracker.logRobotInit();

    		usbCamera = CameraServer.getInstance().startAutomaticCapture("Intake Camera", 0);
    		usbCamera.setResolution(320, 240);
    		usbCamera.setFPS(15);
    		// view camera at http://10.6.86.2:1181?action=stream
    		// use Ctrl-+ to increase size to full screen
    		
    		loopController = new LoopController();
    		loopController.register(drive.getVelocityPIDLoop());
    		loopController.register(DriveLoop.getInstance());
       		loopController.register(RobotStateLoop.getInstance());
    		
    		smartDashboardInteractions = new SmartDashboardInteractions();
    		smartDashboardInteractions.initWithDefaults();
    		
    		// set datalogger and time info
    		TimeZone.setDefault(TimeZone.getTimeZone("America/New_York"));
    		
    		robotLogger = DataLogController.getRobotLogController();
    		robotLogger.register(Drive.getInstance().getLogger());
    		robotLogger.register(drive.getCommand().getLogger());
    		robotLogger.register(DriveState.getInstance().getLogger());
    		robotLogger.register(Superstructure.getInstance().getLogger());
    		robotLogger.register(RobotState.getInstance().getLogger());
    		
    		setInitialPose(new Pose());


    		
    	}
    	catch(Throwable t)
    	{
    		CrashTracker.logThrowableCrash(t);
    		throw t;
    	}
	}
	
	public void setInitialPose (Pose _initialPose){
		robotState.reset(Timer.getFPGATimestamp(), DriveState.getInstance().getLeftDistanceInches(), DriveState.getInstance().getRightDistanceInches(), _initialPose);
    	System.out.println("InitialPose: " + _initialPose);
    }
    
    public void zeroAllSensors()
    {
    	drive.zeroSensors();
    	superStructure.zeroSensors();
		// mSuperstructure.zeroSensors();
    }
    
    public void stopAll()
    {
    	drive.stop();
    	superStructure.stop();
		// mSuperstructure.stop();
    }



	/****************************************************************
	 * DISABLED MODE
	 ****************************************************************/

	@Override
	public void disabledInit()
	{
		operationalMode = OperationalMode.DISABLED;
		boolean logToFile = true;
		boolean logToSmartDashboard = true;
		robotLogger.setOutputMode(logToFile, logToSmartDashboard);

		try
		{
			CrashTracker.logDisabledInit();
			if (autoModeExecuter != null)
			{
				autoModeExecuter.stop();
			}
			autoModeExecuter = null;

			stopAll(); // stop all actuators
			loopController.start();
		}
		catch (Throwable t)
		{
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledPeriodic()
	{
		try
		{
			stopAll(); // stop all actuators

			System.gc(); // runs garbage collector
		}
		catch (Throwable t)
		{
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}



	/****************************************************************
	 * AUTONOMOUS MODE
	 ****************************************************************/

	@Override
	public void autonomousInit() {
    	operationalMode = OperationalMode.AUTONOMOUS;
    	boolean logToFile = true;
    	boolean logToSmartDashboard = true;
    	robotLogger.setOutputMode(logToFile, logToSmartDashboard);

    	try
    	{
			superStructure.enable();

			CrashTracker.logAutoInit();

			if(autoModeExecuter != null){
    			autoModeExecuter.stop();
    		}
    		autoModeExecuter = null;
    		
			autoModeExecuter = new AutoModeExecuter();
			autoModeExecuter.setAutoMode( smartDashboardInteractions.getAutoModeSelection() );

			setInitialPose( autoModeExecuter.getAutoMode().getInitialPose() );
 
			autoModeExecuter.start();
    	}
    	catch(Throwable t)
    	{
    		CrashTracker.logThrowableCrash(t);
    		throw t;
    	}
		
	}

	@Override
	public void autonomousPeriodic() {
    	try
    	{
    		
    	}
    	catch (Throwable t)
    	{
    		CrashTracker.logThrowableCrash(t);
    		throw t;
    	}
	}
	
	
	/****************************************************************
	 * TELEOP MODE
	 ****************************************************************/

	@Override
	public void teleopInit(){
		operationalMode = OperationalMode.TELEOP;
		boolean logToFile = true;
		boolean logToSmartDashboard = true;
		robotLogger.setOutputMode(logToFile, logToSmartDashboard);

		try 
		{
			CrashTracker.logTeleopInit();

			// Select joystick control method
			controls = smartDashboardInteractions.getJoystickControlsMode();

			// Configure looper
			loopController.start();
			superStructure.enable();
			
			if(autoModeExecuter != null){
    			autoModeExecuter.stop();
    		}
    		
			drive.setOpenLoop(DriveCommand.COAST());
		} 
		catch (Throwable t) 
		{
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}
	
	int prevButtonBoardDirection = -1;
	
	@Override
	public void teleopPeriodic() 
	{
		try
		{
			DriveCommand driveCmd = controls.getDriveCommand();
			drive.setOpenLoop(driveCmd);
		}
		catch (Throwable t)
		{
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}



	/****************************************************************
	 * TEST MODE
	 ****************************************************************/

	@Override
	public void testInit() 
	{
		loopController.start();
	}

	@Override
	public void testPeriodic()
	{
		drive.testDriveSpeedControl();
	}
	
	
	// called after disabledPeriodic, autoPeriodic, and teleopPeriodic 
	@Override
	public void robotPeriodic()
	{
		robotLogger.log();
	}


	
	
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
			put("OperationalMode", operationalMode.getVal());
        }
    };
    
    public DataLogger getLogger() { return logger; }
}

