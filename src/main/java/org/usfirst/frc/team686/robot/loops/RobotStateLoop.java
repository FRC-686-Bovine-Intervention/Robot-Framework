package org.usfirst.frc.team686.robot.loops;


import org.usfirst.frc.team686.robot.command_status.DriveState;
import org.usfirst.frc.team686.robot.command_status.RobotState;

import edu.wpi.first.wpilibj.Timer;

/**
 * Periodically estimates the state of the robot using the robot's distance
 * traveled (compares two waypoints), gyroscope orientation, and velocity, among
 * various other factors. Similar to a car's odometer.
 */
public class RobotStateLoop implements Loop 
{
    static RobotStateLoop instance = new RobotStateLoop();
    public static RobotStateLoop getInstance() { return instance; }

    RobotState robotState;
    DriveState DriveState;
    
    RobotStateLoop() 
    {
        robotState = RobotState.getInstance();
        DriveState = DriveState.getInstance();
    }
    


    @Override
    public void onStart() 
    {
    	robotState.setPrevEncoderDistance(DriveState.getLeftDistanceInches(), DriveState.getRightDistanceInches());
    }

    @Override
    public void onLoop() 
    {
    	// the following DriveState elements are set during DriveLoop, called just previous to RobotStateLoop,
    	// and in the same LoopController thread
    	
        double time      = Timer.getFPGATimestamp();
        double lDistance = DriveState.getLeftDistanceInches();
        double rDistance = DriveState.getRightDistanceInches();
        double lSpeed    = DriveState.getLeftSpeedInchesPerSec();
        double rSpeed    = DriveState.getRightSpeedInchesPerSec(); 
        double gyroAngle = DriveState.getHeading();

        robotState.generateOdometryFromSensors(time, lDistance, rDistance, lSpeed, rSpeed, gyroAngle);
    }

    @Override
    public void onStop() 
    {
        // no-op
    }

}
