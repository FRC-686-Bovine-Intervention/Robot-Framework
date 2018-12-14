package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.AutoModeSelector;
import frc.robot.auto.actions.WaitAction;

/**
 * Fallback for when all autonomous modes do not work, resulting in a robot
 * standstill
 */
public class StandStillMode extends AutoModeBase
{

    @Override
    protected void routine() throws AutoModeEndedException
    {
        System.out.print("Starting Stand Still Mode...");
        runAction(new WaitAction(AutoModeSelector.getStartDelay()));
        System.out.println("Done!");
    }
}
