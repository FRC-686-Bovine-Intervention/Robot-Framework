package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.AutoModeSelector;
import frc.robot.auto.actions.*;

/**
 * Just drive in a straight line, using VelocityHeading mode
 */
public class DriveStraightMode extends AutoModeBase
{
   public DriveStraightMode()
    {

    }

    @Override
    protected void routine() throws AutoModeEndedException
    {
        System.out.println("Starting Auto Mode: Drive Straight");

        runAction(new WaitAction(AutoModeSelector.getStartDelay()));
        runAction(new DriveStraightAction(48.0, 12.0));     // drive forward 48" at 12"/sec
    }
}
