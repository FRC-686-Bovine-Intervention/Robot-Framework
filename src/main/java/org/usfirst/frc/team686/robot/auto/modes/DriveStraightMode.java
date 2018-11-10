package org.usfirst.frc.team686.robot.auto.modes;

import org.usfirst.frc.team686.robot.auto.AutoModeBase;
import org.usfirst.frc.team686.robot.auto.AutoModeEndedException;
import org.usfirst.frc.team686.robot.auto.actions.DriveStraightAction;

/**
 * Just drive in a straight line, using VelocityHeading mode
 */
public class DriveStraightMode extends AutoModeBase {

    public DriveStraightMode(int lane, boolean shouldDriveBack) 
    { 
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
    	System.out.println("Starting Auto Mode: Drive Straight");
    	
        runAction(new DriveStraightAction(48.0, 12.0));       		         
    }
}
