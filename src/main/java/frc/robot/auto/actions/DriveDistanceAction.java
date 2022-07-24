package frc.robot.auto.actions;

import frc.robot.command_status.DriveCommand;
import frc.robot.command_status.DriveState;
import frc.robot.subsystems.Drive;



/* This action is used to debug the TurnToAngle action, which also uses Position mode 
   The robot can be up on blocks for these tests */
public class DriveDistanceAction implements Action {

    double lDeltaDistanceInches, rDeltaDistanceInches;

    double distanceThresholdInches = 3.0;
    double speedThresholdInchesPerSec = 6.0;

    double lDistanceTarget, rDistanceTarget;

    public DriveDistanceAction(double _lDeltaDistanceInches, double _rDeltaDistanceInches)
    {
        lDeltaDistanceInches = _lDeltaDistanceInches;
        rDeltaDistanceInches = _rDeltaDistanceInches;
    }

    @Override
    public void start() {
        double lDistanceInches = DriveState.getInstance().getLeftDistanceInches();
        double rDistanceInches = DriveState.getInstance().getRightDistanceInches();

        lDistanceTarget = lDistanceInches + lDeltaDistanceInches;
        rDistanceTarget = rDistanceInches + rDeltaDistanceInches;

        Drive.getInstance().setPositionSetpoint(lDistanceTarget, rDistanceTarget);
    }

    @Override
    public void run() {
        // keep sending this same command so the watchdog doesn't disable the motors
        Drive.getInstance().setPositionSetpoint(lDistanceTarget, rDistanceTarget);
    }

    @Override
    public boolean isFinished() {
        return  (Math.abs(lDistanceTarget - DriveState.getInstance().getLeftDistanceInches()) < distanceThresholdInches) &&
                (Math.abs(rDistanceTarget - DriveState.getInstance().getRightDistanceInches()) < distanceThresholdInches) &&
                (Math.abs(DriveState.getInstance().getLeftSpeedInchesPerSec()) < speedThresholdInchesPerSec) &&
                (Math.abs(DriveState.getInstance().getRightSpeedInchesPerSec()) < speedThresholdInchesPerSec);
}

    @Override
    public void done() {
        Drive.getInstance().setOpenLoop(DriveCommand.BRAKE());  // brake so that we stop near target angle
    }
}
