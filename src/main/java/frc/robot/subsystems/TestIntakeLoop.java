package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants;
import frc.robot.subsystems.TestIntakeStatus.IntakeState;

public class TestIntakeLoop extends LoopBase{
    private static TestIntakeLoop instance;
    public static TestIntakeLoop getInstance() {if(instance == null){instance = new TestIntakeLoop();}return instance;}

    private TestIntake intake;
    private TestIntakeStatus status;

    private final TalonFX ArmMotor;
    private final VictorSPX RollerMotor;
    
    private static final double kGroundHoldingThresholdDegrees = 4.0;
    private static final double kGroundHoldingPercentOutput = -0.25;

    private static final double kGearRatio = 16.0 * 48.0/12.0;  // 16 in gearbox, 48t:12t sprockets
    private static final double kEncoderUnitsPerRev = 2048 * kGearRatio;
    private static final double kEncoderUnitsPerDeg = kEncoderUnitsPerRev/360.0;

    private static final double kP = 0.06;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kMaxVelocityDegPerSecond = 90; //240
    private static final double kMaxAccelerationDegPerSecSquared = 390;

    private static final double kAtTargetThresholdDegrees = 5.0;
    public static final double kClimbingHoldPercent = 0.2;

    private static final double kDisableRecalTimeThreshold = 5;

    private ProfiledPIDController pid;

    public boolean calibrated = false;
    public boolean autoCalibrate = true;

    private NetworkTableEntry statusEntry;
    private NetworkTableEntry armposEntry;
    private NetworkTableEntry calibratedEntry;
    private NetworkTableEntry calibrateButton;
    private SendableChooser<TestIntakeStatus.IntakeState> stateChooser = new SendableChooser<>();
    private NetworkTableEntry armCurrentEntry;
    private NetworkTableEntry armCurrentPosEntry;
    private NetworkTableEntry armPIDOutputEntry;
    private NetworkTableEntry armGoalEntry;
    
    private TestIntakeLoop()
    {
        status = TestIntakeStatus.getInstance();
        _status = status;

        ArmMotor = new TalonFX(Constants.kArmMotorID);
        RollerMotor = new VictorSPX(Constants.kRollerMotorID);
    
        ArmMotor.configFactoryDefault();
        ArmMotor.setInverted(TalonFXInvertType.CounterClockwise);
        ArmMotor.setNeutralMode(NeutralMode.Brake);
        ArmMotor.configForwardSoftLimitThreshold(degreesToEncoderUnits(TestIntakeStatus.IntakeState.DEFENSE.armPos.angleDeg));

        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(kMaxVelocityDegPerSecond, kMaxAccelerationDegPerSecSquared);
        pid = new ProfiledPIDController(kP, kI, kD, constraints);
        pid.reset(TestIntakeStatus.ArmPosEnum.CALIBRATION.angleDeg);

        for (TestIntakeStatus.IntakeState s : TestIntakeStatus.IntakeState.values()) {stateChooser.addOption(s.name(), s);}
    
        tab = Shuffleboard.getTab("Intake");
        enabledEntry = tab.add("Enable", true).withWidget(BuiltInWidgets.kToggleSwitch)                     .withPosition(0,3).getEntry();
        statusEntry = tab.add("Status", "not updating what").withWidget(BuiltInWidgets.kTextView)           .withPosition(0,0).withSize(2,1).getEntry();
        tab.add("State Chooser", stateChooser)                                                              .withPosition(0,4).withSize(2,1);
        calibratedEntry = tab.add("Calibrated", false).withWidget(BuiltInWidgets.kBooleanBox)               .withPosition(1,1).getEntry();
        calibrateButton = tab.add("Calibrate", false).withWidget(BuiltInWidgets.kToggleButton)              .withPosition(1,3).getEntry();
        armposEntry = tab.add("Arm position", "not updating what").withWidget(BuiltInWidgets.kTextView)     .withPosition(0,1).getEntry();
        armCurrentEntry = tab.add("Arm Current", -9999).withWidget(BuiltInWidgets.kTextView)                .withPosition(8,0).getEntry();
        armCurrentPosEntry = tab.add("Arm Current Pos", -9999).withWidget(BuiltInWidgets.kTextView)         .withPosition(9,0).getEntry();
        armPIDOutputEntry = tab.add("Arm PID Output", -9999).withWidget(BuiltInWidgets.kTextView)           .withPosition(8,1).getEntry();
        armGoalEntry = tab.add("Arm PID Goal", -9999).withWidget(BuiltInWidgets.kTextView)                  .withPosition(9,1).getEntry();
    }

    // @Override
    // public void Enabled() {
    //     ArmMotor.configForwardSoftLimitEnable(true);
    //     ArmMotor.setNeutralMode(NeutralMode.Brake);
    //     autoCalibrate = !DriverStation.isTest();
    //     if(autoCalibrate && !calibrated) {runCalibration();}
    //     switch (intakeStatus)
    //     {
    //         case DEFENSE: default:
    //             RollerMotor.set(VictorSPXControlMode.PercentOutput, 0);
    //             setTargetPos(TestIntakeStatus.ArmPosEnum.RAISED);
    //         break;
    //         case INTAKE:
    //             if(isAtPos(TestIntakeStatus.ArmPosEnum.LOWERED, 30)) {RollerMotor.set(VictorSPXControlMode.PercentOutput, kIntakePercentOutput);}
    //             setTargetPos(TestIntakeStatus.ArmPosEnum.LOWERED);
    //         break;
    //         case OUTTAKE:
    //             if(isAtPos(TestIntakeStatus.ArmPosEnum.RAISED)) {RollerMotor.set(VictorSPXControlMode.PercentOutput, kOuttakePercentOutput);}
    //             setTargetPos(TestIntakeStatus.ArmPosEnum.RAISED);
    //         break;
    //         case OUTTAKE_GROUND:
    //             if(isAtPos(TestIntakeStatus.ArmPosEnum.LOWERED)) {RollerMotor.set(VictorSPXControlMode.PercentOutput, kOuttakePercentOutput);}
    //             setTargetPos(TestIntakeStatus.ArmPosEnum.LOWERED);
    //         break;
    //         case CLIMBING:
    //             RollerMotor.set(VictorSPXControlMode.PercentOutput, 0);
    //             ArmMotor.set(TalonFXControlMode.PercentOutput, climbingPower);
    //             pid.reset(encoderUnitsToDegrees(ArmMotor.getSelectedSensorPosition()));
    //         break;
    //         case HARD_STOPS:
    //             RollerMotor.set(VictorSPXControlMode.PercentOutput, 0);
    //             setTargetPos(TestIntakeStatus.ArmPosEnum.HARD_STOPS);
    //         break;
    //         case CALIBRATING:
    //             ArmMotor.configForwardSoftLimitEnable(false);
    //             pid.reset(encoderUnitsToDegrees(ArmMotor.getSelectedSensorPosition()));
    //             calibrated = false;
    //             ArmMotor.set(TalonFXControlMode.PercentOutput, kCalibrationPercentOutput);
    //         break;
    //     }
        
    //     if (checkFwdLimitSwitch())
    //     {
    //         ArmMotor.setSelectedSensorPosition(degreesToEncoderUnits(TestIntakeStatus.ArmPosEnum.CALIBRATION.angleDeg));
    //         calibrated = true;
    //         setState(TestIntakeStatus.IntakeState.DEFENSE);
    //         if (!prevFwdLimitSwitchClosed)
    //         {
    //             ArmMotor.set(TalonFXControlMode.PercentOutput, 0);
    //             pid.reset(calState);
    //             pid.setGoal(calState);
    //         }
    //         prevFwdLimitSwitchClosed = checkFwdLimitSwitch();
    //     }
    // }

    @Override
    public void Enabled()
    {
        sendCommands();
    }

    private void sendCommands()
    {
        ArmMotor.setNeutralMode(NeutralMode.Brake);

        TestIntakeCommand intakeCmd = intake.getIntakeCommand();

        if(autoCalibrate && !calibrated)
        {
            intakeCmd = TestIntakeCommand.fromState(IntakeState.CALIBRATING); //Calibrate if not calibrated
        }
        if(intakeCmd == null) {intakeCmd = TestIntakeCommand.fromState(IntakeState.DEFENSE);} //If null default to defense
        if(intakeCmd.usingPosition()) //Set position
        {
            if(isAtPos(intakeCmd.getArmPosition(), 30))
            {
                RollerMotor.set(ControlMode.PercentOutput, intakeCmd.getRollerSpeed());
            }
            setTargetPos(intakeCmd.getArmPosition());
        }
        else //Use percent output
        {
            RollerMotor.set(ControlMode.PercentOutput, intakeCmd.getRollerSpeed());
            ArmMotor.set(TalonFXControlMode.PercentOutput, intakeCmd.getPercentOutput());
            pid.reset(encoderUnitsToDegrees(ArmMotor.getSelectedSensorPosition()));
        }
        intakeStatus = intakeCmd.getState();

        if (checkFwdLimitSwitch())
        {
            ArmMotor.setSelectedSensorPosition(degreesToEncoderUnits(TestIntakeStatus.ArmPosEnum.CALIBRATION.angleDeg));
            calibrated = true;
            if (!prevFwdLimitSwitchClosed)
            {
                ArmMotor.set(TalonFXControlMode.PercentOutput, 0);
                pid.reset(TestIntakeStatus.ArmPosEnum.CALIBRATION.angleDeg);
                pid.setGoal(TestIntakeStatus.ArmPosEnum.CALIBRATION.angleDeg);
            }
            prevFwdLimitSwitchClosed = checkFwdLimitSwitch();
        }
    }

    private double disabledTime;
    @Override
    public void Disabled() {
        if(_status.Enabled.IsInitState) disabledTime = Timer.getFPGATimestamp();
        ArmMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        RollerMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
        pid.reset(encoderUnitsToDegrees(ArmMotor.getSelectedSensorPosition()));

        if(Timer.getFPGATimestamp() - disabledTime > kDisableRecalTimeThreshold)
        {
            calibrated = false;
            ArmMotor.setNeutralMode(NeutralMode.Coast);
        }
    }
    @Override
    public void Update() {
        intake = TestIntake.getInstance();
        if(calibrateButton.getBoolean(false))
        {
            calibrateButton.setBoolean(false);
            runCalibration();
        }
        status.setStatus(intakeStatus)
              .setArmTargetPos(targetPos)
              .setCalibrated(calibrated)
              .setArmCurrentPos(encoderUnitsToDegrees(ArmMotor.getSelectedSensorPosition()))
              .setArmMotorCurrent(ArmMotor.getStatorCurrent())
              .setArmPIDGoal(pid.getGoal().position)
              .setArmPIDOutput(pidOutput);

        statusEntry.setString(status.getStatus().name());
        armposEntry.setString(status.getArmTargetPos().name());
        calibratedEntry.setBoolean(status.getCalibrated());
        armCurrentPosEntry.setDouble(status.getArmCurrentPos());
        armCurrentEntry.setDouble(status.getArmMotorCurrent());
        armGoalEntry.setDouble(status.getArmPIDGoal());
        armPIDOutputEntry.setDouble(status.getArmPIDOutput());
    }

    public TestIntakeStatus.ArmPosEnum targetPos = TestIntakeStatus.ArmPosEnum.RAISED;
    
    public TestIntakeStatus.IntakeState intakeStatus = TestIntakeStatus.IntakeState.DEFENSE;

    // @Override
    // public void runTestMode()
    // {
    //     if (stateChooser.getSelected() != null) intakeStatus = stateChooser.getSelected();
    //     if (calibrateButton.getBoolean(false))
    //     {
    //         calibrateButton.setBoolean(false);
    //         runCalibration();
    //     }
    //     autoCalibrate = false;
    //     run();
    //     autoCalibrate = true;
    // }
    public void runCalibration()
    {
        calibrated = false;
    }

    public boolean isAtPos(TestIntakeStatus.ArmPosEnum pos, double threshold)
    {
        double currentAngleDegrees = encoderUnitsToDegrees(ArmMotor.getSelectedSensorPosition());

        return (Math.abs(currentAngleDegrees - pos.angleDeg) < threshold);
    }

    public boolean isAtPos(TestIntakeStatus.ArmPosEnum pos) {return isAtPos(pos,kAtTargetThresholdDegrees);}

    public void setTargetPos(TestIntakeStatus.ArmPosEnum pos)
    {
        targetPos = pos;
        setPos(targetPos);
    }

    // public TestIntakeStatus.ArmPosEnum getClosestPos(double pos)
    // {
    //     if(Math.abs(TestIntakeStatus.ArmPosEnum.RAISED.angleDeg - pos) < threshold)
    //     {

    //     }
    // }

    double pidOutput = 0.0;
    private void setPos(TestIntakeStatus.ArmPosEnum pos)
    {
        pid.setGoal(pos.angleDeg);
        pidOutput = 0.0;
        double currentAngleDegrees = encoderUnitsToDegrees(ArmMotor.getSelectedSensorPosition());

        pidOutput = pid.calculate(currentAngleDegrees);
        ArmMotor.set(TalonFXControlMode.PercentOutput, pidOutput);

        if ((pid.getGoal().position == TestIntakeStatus.ArmPosEnum.LOWERED.angleDeg) && (currentAngleDegrees < kGroundHoldingThresholdDegrees))
        {
            ArmMotor.set(TalonFXControlMode.PercentOutput, kGroundHoldingPercentOutput);
        }
    }

    public static int degreesToEncoderUnits(double _degrees) {return (int)(_degrees * kEncoderUnitsPerDeg);}
    public static double encoderUnitsToDegrees(double _encoderUnits) {return (double)(_encoderUnits / kEncoderUnitsPerDeg);}


    private boolean prevFwdLimitSwitchClosed;
    public boolean checkFwdLimitSwitch()
    {
        boolean fwdLimitSwitchClosed = (ArmMotor.isFwdLimitSwitchClosed() == 1);
        return fwdLimitSwitchClosed;
    }
}
