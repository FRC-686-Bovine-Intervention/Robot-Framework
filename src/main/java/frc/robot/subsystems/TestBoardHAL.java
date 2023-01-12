package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.RobotBase;

public class TestBoardHAL extends HALBase {
    private static TestBoardHAL instance;
    public static TestBoardHAL getInstance() {if(instance == null){instance = new TestBoardHAL();} return instance;}

    private final TalonFX LeftMotor;
    private final TalonFX RightMotor;

    private TestBoardHAL()
    {
        if(RobotBase.isReal())
        {
            LeftMotor = new TalonFX(5); 
            RightMotor = null;//new TalonFX(Constants.kRightClimberID);
        }
        else
        {
            LeftMotor = null;
            RightMotor = null;
        }

        if(LeftMotor != null)
        {
            LeftMotor.configFactoryDefault();
            LeftMotor.setInverted(TalonFXInvertType.Clockwise);
            LeftMotor.configOpenloopRamp(0.75);
        }

        if(RightMotor != null)
        {
            RightMotor.configFactoryDefault();
            RightMotor.setInverted(TalonFXInvertType.CounterClockwise);
            if(LeftMotor != null)
                RightMotor.follow(LeftMotor);
            else
                RightMotor.configOpenloopRamp(0.75);
        }
    }

    public TestBoardHAL setMotorPower(double power)
    {
        if(LeftMotor != null)
            LeftMotor.set(TalonFXControlMode.PercentOutput, power);
        else if(RightMotor != null)
            RightMotor.set(TalonFXControlMode.PercentOutput, power);
        return this;
    }

    public double getStatorCurrent() {return getStatorCurrent(0);}
    public double getStatorCurrent(double defaultVal)
    {
        double r = defaultVal;
        if(LeftMotor != null)
            r = LeftMotor.getStatorCurrent();
        else if(RightMotor != null)
            r = RightMotor.getStatorCurrent();
        return r;
    }

    public double getMotorOutputVoltage() {return getMotorOutputVoltage(0);}
    public double getMotorOutputVoltage(double defaultVal)
    {
        double r = defaultVal;
        if(LeftMotor != null)
            r = LeftMotor.getMotorOutputVoltage();
        else if(RightMotor != null)
            r = RightMotor.getMotorOutputVoltage();
        return r;
    }
}
