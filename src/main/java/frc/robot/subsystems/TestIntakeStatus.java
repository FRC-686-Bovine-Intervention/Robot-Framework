package frc.robot.subsystems;

public class TestIntakeStatus extends StatusBase{
    private static TestIntakeStatus instance;
    public static TestIntakeStatus getInstance() {if(instance == null){instance = new TestIntakeStatus();}return instance;}

    public enum IntakeState {
        DEFENSE(ArmPosEnum.RAISED,0),
        INTAKE(ArmPosEnum.LOWERED,0.7),
        OUTTAKE(ArmPosEnum.RAISED,-1),
        OUTTAKE_GROUND(ArmPosEnum.LOWERED,-1),
        CLIMBING(null,0),
        HARD_STOPS(ArmPosEnum.HARD_STOPS,0),
        CALIBRATING(ArmPosEnum.CALIBRATION,0),
        CUSTOM(null,0);

        public final ArmPosEnum armPos;
        public final double rollSpeed;
        IntakeState(ArmPosEnum armPos, double rollSpeed)
        {
            this.armPos = armPos;
            this.rollSpeed = rollSpeed;
        }
    }
    public enum ArmPosEnum {
        LOWERED(0),
        RAISED(105),
        HARD_STOPS(55),
        CALIBRATION(112);

        public final double angleDeg;
        ArmPosEnum(double angleDeg) {this.angleDeg = angleDeg;}
    }

    private IntakeState status;
    private boolean calibrated;
    private double armPIDOutput;
    private double armPIDGoal;
    private double armCurrentPos;
    private double armMotorCurrent;
    private ArmPosEnum armTargetPos;

    public TestIntakeStatus setStatus(IntakeState status)               {this.status = status; return this;}
    public IntakeState getStatus()                                      {return status;}

    public TestIntakeStatus setCalibrated(boolean calibrated)           {this.calibrated = calibrated; return this;}
    public boolean getCalibrated()                                      {return calibrated;}

    public TestIntakeStatus setArmPIDOutput(double armPIDOutput)        {this.armPIDOutput = armPIDOutput; return this;}
    public double getArmPIDOutput()                                     {return armPIDOutput;}

    public TestIntakeStatus setArmPIDGoal(double armPIDGoal)            {this.armPIDGoal = armPIDGoal; return this;}
    public double getArmPIDGoal()                                       {return armPIDGoal;}

    public TestIntakeStatus setArmCurrentPos(double armCurrentPos)      {this.armCurrentPos = armCurrentPos; return this;}
    public double getArmCurrentPos()                                    {return armCurrentPos;}

    public TestIntakeStatus setArmMotorCurrent(double armMotorCurrent)  {this.armMotorCurrent = armMotorCurrent; return this;}
    public double getArmMotorCurrent()                                  {return armMotorCurrent;}

    public TestIntakeStatus setArmTargetPos(ArmPosEnum armTargetPos)    {this.armTargetPos = armTargetPos; return this;}
    public ArmPosEnum getArmTargetPos()                                 {return armTargetPos;}
}
