/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.AutoModeExecuter;
import frc.robot.auto.AutoModeSelector;
import frc.robot.command_status.DriveCommand;
import frc.robot.command_status.DriveState;
import frc.robot.command_status.RobotState;
import frc.robot.lib.joystick.ArcadeDriveJoystick;
import frc.robot.lib.joystick.ButtonBoard;
import frc.robot.lib.joystick.JoystickControlsBase;
import frc.robot.lib.joystick.JoystickSelector;
import frc.robot.lib.sensors.Limelight;
import frc.robot.lib.util.CrashTracker;
import frc.robot.lib.util.Pose;
import frc.robot.loops.DriveLoop;
import frc.robot.loops.LoopController;
import frc.robot.loops.RobotStateLoop;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleMotionMagicSubsystem;
import frc.robot.subsystems.Superstructure;


public class Robot extends IterativeRobot
{
	private LoopController mEnabledLooper = new LoopController();
	private LoopController mDisabledLooper = new LoopController();
	JoystickControlsBase joystick = ArcadeDriveJoystick.getInstance();	// can be changed every teleopInit()
	ButtonBoard buttonBoard = ButtonBoard.getInstance();

	// private final SubsystemManager subsystemManager = new SubsystemManager (
	// 	Arrays.asList(
	// 		RobotStateEstimator.getInstance(),
	// 		Drive.getInstance(),
	// 		Superstructure.getInstance(),
	// 		Intake.getInstance(),
	// 		Elevator.getInstance(),
	// 		Infrastructure.getInstance()
	// 	)
	// );

    private AutoModeSelector autoModeSelector = AutoModeSelector.getInstance();
    private JoystickSelector joystickSelector = JoystickSelector.getInstance();

	// Two camera options
	Limelight limelight = new Limelight();
	UsbCamera usbCamera;

	AutoModeExecuter autoModeExecuter = null;
	LoopController loopController;

	Superstructure superStructure = Superstructure.getInstance();
	RobotState robotState = RobotState.getInstance();
	Drive drive = Drive.getInstance();
	ExampleMotionMagicSubsystem exampleMotionMagicSubsystem = ExampleMotionMagicSubsystem.getInstance(); 



	public Robot()
	{
		CrashTracker.logRobotConstruction();
	}

	@Override
	public void robotInit()
	{
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

			setInitialPose(new Pose());

		}
		catch (Throwable t)
		{
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	public void setInitialPose(Pose _initialPose)
	{
		robotState.reset(Timer.getFPGATimestamp(), DriveState.getInstance().getLeftDistanceInches(), 
		                                           DriveState.getInstance().getRightDistanceInches(), _initialPose);
		System.out.println("InitialPose: " + _initialPose);
	}

	public void zeroAllSensors()
	{
		drive.zeroSensors();
		superStructure.zeroSensors();
		// mSuperstructure.zeroSensors();
	}

	

	/****************************************************************
	 * DISABLED/AUTONOMOUS/TELEOP INITS
	 ****************************************************************/

	@Override
	public void disabledInit() 
	{
		SmartDashboard.putString("Match Cycle", "DISABLED");
		
		try 
		{
			CrashTracker.logDisabledInit();
			mEnabledLooper.stop();

			// kill any autoModeExecuters
			if (autoModeExecuter != null) {
				autoModeExecuter.stop();
			}
			autoModeExecuter = null;

			// zero subsystems if at limits
			exampleMotionMagicSubsystem.zeroIfAtLimit();
			   
		   	mDisabledLooper.start();
		}
		catch (Throwable t)
		{
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}
	
	
	@Override
	public void autonomousInit()
	{
		SmartDashboard.putString("Match Cycle", "AUTONOMOUS");
		
		try
		{
			CrashTracker.logAutoInit();
			mDisabledLooper.stop();
			if (autoModeExecuter != null) {
				autoModeExecuter.stop();
			}
			autoModeExecuter = null;

			limelight.autoInit();
			
			autoModeExecuter = new AutoModeExecuter();
			autoModeExecuter.setAutoMode(autoModeSelector.getAutoModeSelection());
			
			setInitialPose(autoModeExecuter.getAutoMode().getInitialPose());
			
			autoModeExecuter.start();
			mEnabledLooper.start();			
		}
		catch (Throwable t)
		{
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
		
	}
	
	@Override
	public void teleopInit()
	{
		SmartDashboard.putString("Match Cycle", "AUTONOMOUS");
		
		try
		{
			CrashTracker.logTeleopInit();
			mDisabledLooper.stop();
			if (autoModeExecuter != null) {
				autoModeExecuter.stop();
			}
			
			limelight.teleopInit();
			
			// Select joystick control method
			joystick = joystickSelector.getJoystickControlsMode();
			
			drive.setOpenLoop(DriveCommand.COAST());
			mEnabledLooper.start();
		}
		catch (Throwable t)
		{
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}




	/****************************************************************
	 * DISABLED MODE
	 ****************************************************************/

	 @Override
	public void disabledPeriodic()
	{
		try
		{
			limelight.disabledPeriodic();
			
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
	public void autonomousPeriodic()
	{
		try
		{
			// do nothing (AutoModeExecuter thread is operational)
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
	public void teleopPeriodic()
	{
		try
		{
			DriveCommand driveCmd = joystick.getDriveCommand();
			drive.setOpenLoop(driveCmd);
		}
		catch (Throwable t)
		{
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}
	
	/****************************************************************
	 * robotPeriodic
	 * called after disabledPeriodic, autoPeriodic, and teleopPeriodic
	 ****************************************************************/
	
	@Override
	public void robotPeriodic()
	{
		outputToSmartDashboard();
	}
	
	
	
	public void outputToSmartDashboard() 
	{
		RobotState.getInstance().outputToSmartDashboard();
        Drive.getInstance().outputTelemetry();
        //mAutoFieldState.outputToSmartDashboard();
        //mAutoModeSelector.outputToSmartDashboard();
        mEnabledLooper.outputToSmartDashboard();
        // SmartDashboard.updateValues();
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

}	
