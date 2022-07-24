package frc.robot.auto.actions;

import frc.robot.command_status.DriveCommand;
import frc.robot.subsystems.Drive;

public class TurnToAngleAction implements Action{

    private double targetAngleDeg;

    private static final double angleErrorThresholdDeg = 1.0;
    private static final double distanceThresholdInches = 1.0;
    private static final double wheelVelocityThresholdInchesPerSec = 5.0;

    public TurnToAngleAction(double _targetAngleDeg)
    {
        targetAngleDeg = _targetAngleDeg;
    }

    @Override
    public void start() {
        Drive.getInstance().setTurnToHeadingSetpoint(targetAngleDeg);
    }

    @Override
    public void run() {
        // keep sending same drive commands
        DriveCommand driveCmd = Drive.getInstance().getCommand();
        driveCmd.setMotors(driveCmd.getLeftMotor(), driveCmd.getRightMotor());
    }

    @Override
    public boolean isFinished() {
        boolean finished = Drive.getInstance().isTurnToHeadingFinished(distanceThresholdInches);
        if (finished) {
            //DEBUG
            System.out.println("Done with TurnToAngleAction");         
        }
        return finished;
    }

    @Override
    public void done() {
        Drive.getInstance().setOpenLoop(DriveCommand.BRAKE());  // brake so that we stop near target angle
    }
}
