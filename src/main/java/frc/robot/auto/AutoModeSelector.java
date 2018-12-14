package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.modes.FieldDimensions.StartingPosition;
import frc.robot.auto.modes.*;
import frc.robot.lib.util.Pose;

/**
 * Controls the interactive elements of SmartDashboard.
 *
 * Keeps the network tables keys in one spot and enforces autonomous mode
 * invariants.
 */
public class AutoModeSelector
{
	// singleton class
	private static AutoModeSelector instance = null;

	public static AutoModeSelector getInstance()
	{
		if (instance == null)
		{
			instance = new AutoModeSelector();
		}
		return instance;
	}

    public enum StartDelayOption
    {
        DELAY_0_SEC("0 Sec", 0.0), 
        DELAY_1_SEC("1 Sec", 1.0), 
        DELAY_2_SEC("2 Sec", 2.0), 
        DELAY_3_SEC("3 Sec", 3.0), 
        DELAY_4_SEC("4 Sec", 4.0), 
        DELAY_5_SEC("5 Sec", 5.0);

        public final String name;
        public final double delaySec;

        StartDelayOption(String name, double delaySec)
        {
            this.name = name;
            this.delaySec = delaySec;
        }
    }

    enum AutoModeOption
    {
        STAND_STILL,
        DRIVE_STRAIGHT;
    }

    
    static SendableChooser<StartingPosition> startingPositionChooser;
    static SendableChooser<StartDelayOption> startDelayChooser;
    static SendableChooser<AutoModeOption> autoModeChooser;

    
    public AutoModeSelector()
    {
        startingPositionChooser = new SendableChooser<StartingPosition>();
        startingPositionChooser.addDefault("Left",  StartingPosition.LEFT);
        startingPositionChooser.addObject("Center", StartingPosition.CENTER);
        startingPositionChooser.addObject("Right",  StartingPosition.RIGHT);
        SmartDashboard.putData("Start Position", startingPositionChooser);

        startDelayChooser = new SendableChooser<StartDelayOption>();
        startDelayChooser.addDefault(StartDelayOption.DELAY_0_SEC.toString(), StartDelayOption.DELAY_0_SEC);
        startDelayChooser.addObject(StartDelayOption.DELAY_1_SEC.toString(), StartDelayOption.DELAY_1_SEC);
        startDelayChooser.addObject(StartDelayOption.DELAY_2_SEC.toString(), StartDelayOption.DELAY_2_SEC);
        startDelayChooser.addObject(StartDelayOption.DELAY_3_SEC.toString(), StartDelayOption.DELAY_3_SEC);
        startDelayChooser.addObject(StartDelayOption.DELAY_4_SEC.toString(), StartDelayOption.DELAY_4_SEC);
        startDelayChooser.addObject(StartDelayOption.DELAY_5_SEC.toString(), StartDelayOption.DELAY_5_SEC);
        SmartDashboard.putData("Auto Start Delay", startDelayChooser);

        autoModeChooser = new SendableChooser<AutoModeOption>();
        autoModeChooser.addObject("Stand Still",    AutoModeOption.STAND_STILL);
        autoModeChooser.addObject("Drive Straight", AutoModeOption.DRIVE_STRAIGHT);
        SmartDashboard.putData("Auto Mode", autoModeChooser);
    }

    public Pose getStartPosition()
    {
        StartingPosition startingPosition = (StartingPosition) startingPositionChooser.getSelected();
        return startingPosition.initialPose;
    }


    public double getStartDelay()
    {
        StartDelayOption startDelay = (StartDelayOption) startDelayChooser.getSelected();
        return startDelay.delaySec;
    }


    public AutoModeBase getAutoModeSelection()
    {
        AutoModeOption autoMode = (AutoModeOption) autoModeChooser.getSelected();

        switch (autoMode)
        {
            case STAND_STILL:
            return new StandStillMode();

            case DRIVE_STRAIGHT:
            return new DriveStraightMode();

        default:
            System.out.println("ERROR: unexpected auto mode: " + autoMode);
            return new StandStillMode();
        }
    }

}
