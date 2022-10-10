package frc.robot.subsystems;

import frc.robot.subsystems.TestIntakeStatus.IntakeState;

public class TestIntakeCommand {
    private TestIntakeStatus.IntakeState state = IntakeState.CUSTOM;
    private boolean calibrate;
    private double rollerSpeed;
    private double percentOutput;
    private TestIntakeStatus.ArmPosEnum armPosition;
    private boolean usingPosition = false;

    public TestIntakeCommand(){}

    public static TestIntakeCommand fromPosition(TestIntakeStatus.ArmPosEnum armPosition)
    {
        return fromPosition(armPosition, 0);
    }
    public static TestIntakeCommand fromPosition(TestIntakeStatus.ArmPosEnum armPosition, double rollerSpeed)
    {
        TestIntakeCommand r = new TestIntakeCommand();
        r.setArmPosition(armPosition);
        r.setRollerSpeed(rollerSpeed);
        return r;
    }

    public static TestIntakeCommand fromPercentOutput(double percentOutput)
    {
        return fromPercentOutput(percentOutput, 0);
    }
    public static TestIntakeCommand fromPercentOutput(double percentOutput, double rollerSpeed)
    {
        TestIntakeCommand r = new TestIntakeCommand();
        r.setPercentOutput(percentOutput);
        r.setRollerSpeed(rollerSpeed);
        return r;
    }

    public static TestIntakeCommand fromState(TestIntakeStatus.IntakeState intakeState)
    {
        TestIntakeCommand r = new TestIntakeCommand();
        r.setState(intakeState);
        switch(intakeState)
        {
            case CALIBRATING:
                r.setPercentOutput(0.2);
            break;
            case CLIMBING:
                r.setPercentOutput(0);
            break;
            default:
                r.setArmPosition(intakeState.armPos);
                r.setRollerSpeed(intakeState.rollSpeed);
            break;
        }
        return r;
    }

    public static TestIntakeCommand fromButtons(boolean intakeButton, boolean outtakeButton)
    {
        IntakeState s = IntakeState.DEFENSE;
        if(intakeButton && outtakeButton)
        {
            s = IntakeState.OUTTAKE_GROUND;
        }
        else if(intakeButton)
        {
            s = IntakeState.INTAKE;
        }
        else if(outtakeButton)
        {
            s = IntakeState.OUTTAKE;
        }
        return fromState(s);
    }

    public TestIntakeCommand setState(TestIntakeStatus.IntakeState state)               {this.state = state; return this;}
    public TestIntakeStatus.IntakeState getState()                                      {return state;}

    public TestIntakeCommand setArmPosition(TestIntakeStatus.ArmPosEnum armPosition)    {this.armPosition = armPosition; this.usingPosition = true; return this;}
    public TestIntakeStatus.ArmPosEnum getArmPosition()                                 {return armPosition;}

    public TestIntakeCommand setRollerSpeed(double rollerSpeed)                         {this.rollerSpeed = rollerSpeed; return this;}
    public double getRollerSpeed()                                                      {return rollerSpeed;}

    public TestIntakeCommand setPercentOutput(double percentOutput)                     {this.percentOutput = percentOutput; this.usingPosition = false; return this;}
    public double getPercentOutput()                                                    {return percentOutput;}

    public TestIntakeCommand setCalibrate(boolean calibrate)                            {this.calibrate = calibrate; return this;}
    public boolean getCalibrate()                                                       {return calibrate;}

    public TestIntakeCommand setUsingPosition(boolean usingPosition)                    {this.usingPosition = usingPosition; return this;}
    public boolean usingPosition()                                                      {return usingPosition;}
}
