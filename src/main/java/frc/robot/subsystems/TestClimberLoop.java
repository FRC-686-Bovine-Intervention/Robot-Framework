package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;
import frc.robot.subsystems.TestClimberStatus.ClimberPos;
import frc.robot.subsystems.TestClimberStatus.ClimberState;
import frc.robot.subsystems.TestIntakeStatus.IntakeState;

public class TestClimberLoop extends LoopBase {
    private static TestClimberLoop instance;
    public static TestClimberLoop getInstance() {if(instance == null){instance = new TestClimberLoop();}return instance;}

    //Subsystem group
    private TestClimberStatus status;
    private TestClimber climber;

    //Hardware
    private final TalonFX LeftMotor; 
    private final TalonFX RightMotor;
    private final DigitalInput CalibrationHallEffect;

    //Runtime variables initialization
    private boolean calibrated = false;
    private boolean autoCalibrate = true;
    private boolean calibrationPaused = false;
    private boolean moveToClimbingMode = false;
    private double intakePower = 0;
    private TestClimberStatus.ClimberState climberStatus = TestClimberStatus.ClimberState.DEFENSE;
    private ArrayList<TestClimberStatus.ClimberState> ClimberStatusHistory = new ArrayList<>();
    private TestIntakeCommand intakeCmd;

    //Misc Constants
    private static final double kDefensePower = -0.07;
    private static final double kCalibratingPercent = -0.2;
    private static final double kCalibratingThreshold = 20;
    private static final double kDisableRecalTimeThreshold = 5;
    private static final double kReverseSoftLimit = -3;

    //Rotational constants and functions
    private static final double kShaftCircum = 0.5*Math.PI*26.75/23.58;
    private static final double kGearRatio = 5;
    private static final double kEncoderUnitsPerRev = 2048 * kGearRatio;
    private static final double kEncoderUnitsPerIn = kEncoderUnitsPerRev / kShaftCircum;
    private static int inchesToEncoderUnits(double _degrees) {return (int)(_degrees * kEncoderUnitsPerIn);}
    private static double encoderUnitsToInches(double _encoderUnits) {return (double)(_encoderUnits / kEncoderUnitsPerIn);}

    //Positional constants and functions
    private static final double kAtTargetThresholdInches = 1;
    public boolean isAtPos(ClimberPos pos, double threshold) 
    { 
        double currentDistanceInches = encoderUnitsToInches(LeftMotor.getSelectedSensorPosition()); 
        double targetInches = pos.distIn; 
 
        return (Math.abs(currentDistanceInches - targetInches) < threshold); 
    } 
    public boolean isAtPos(ClimberPos pos) {return isAtPos(pos,kAtTargetThresholdInches);} 

    private static final double kIntakeMaxPercent = 0.3;

    //Shuffleboard widgets
    private NetworkTableEntry statusEntry;
    private NetworkTableEntry historyEntry;
    private NetworkTableEntry calibratedEntry;
    private NetworkTableEntry pauseEntry;

    private NetworkTableEntry calibrateButton;

    private NetworkTableEntry climbingIntakeInput;
    private NetworkTableEntry climberCurrentPosEntry;
    private NetworkTableEntry climberSupplyEntry;
    private NetworkTableEntry climberStatorEntry;
    private NetworkTableEntry climberBusVoltageEntry;
    private NetworkTableEntry climberOutputVoltageEntry;

    private NetworkTableEntry lowbarEntry;
    private NetworkTableEntry extendGroundEntry;
    private NetworkTableEntry slowDriveEntry;
    private NetworkTableEntry retractExtendEntry;
    private NetworkTableEntry intakeEntry;

    private int prevStateChange = 0;

    private TestClimberLoop()
    {
        status = TestClimberStatus.getInstance();
        _status = status;

        LeftMotor = new TalonFX(Constants.kLeftClimberID); 
        RightMotor = new TalonFX(Constants.kRightClimberID);
        CalibrationHallEffect = new DigitalInput(Constants.kClimberHallEffectPort);

        LeftMotor.configFactoryDefault();
        LeftMotor.configOpenloopRamp(0.75);
        //LeftMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 50, 60, 0.25));
        LeftMotor.setInverted(TalonFXInvertType.Clockwise);
        LeftMotor.configForwardSoftLimitThreshold(inchesToEncoderUnits(ClimberPos.EXTENDED.distIn));
        LeftMotor.configForwardSoftLimitEnable(true);
        LeftMotor.configReverseSoftLimitThreshold(inchesToEncoderUnits(kReverseSoftLimit));
        LeftMotor.configReverseSoftLimitEnable(true);
        
        RightMotor.configFactoryDefault();
        RightMotor.setInverted(TalonFXInvertType.CounterClockwise); 
        RightMotor.follow(LeftMotor); 

        setState(ClimberState.DEFENSE); 

        tab = Shuffleboard.getTab("Climber");
        enabledEntry = tab.add("Enable", true)                          .withWidget(BuiltInWidgets.kToggleSwitch)   .withPosition(0,3).getEntry(); 

        statusEntry = tab.add("Status", "not updating")                 .withWidget(BuiltInWidgets.kTextView)       .withPosition(0,0).withSize(2,1).getEntry();
        historyEntry = tab.add("Status History", "not updating")        .withWidget(BuiltInWidgets.kTextView)       .withPosition(3,1).withSize(4,1).getEntry();
        calibratedEntry = tab.add("Calibrated", false)                  .withWidget(BuiltInWidgets.kBooleanBox)     .withPosition(1,1).getEntry();
        pauseEntry = tab.add("Calibration Paused", false)               .withWidget(BuiltInWidgets.kBooleanBox)     .withPosition(0,1).getEntry();

        calibrateButton = tab.add("Calibrate", false)                   .withWidget(BuiltInWidgets.kToggleButton)   .withPosition(1,3).getEntry();

        climbingIntakeInput         = tab.add("Intake Input", -9999)    .withWidget(BuiltInWidgets.kTextView)       .withPosition(8,0).getEntry();
        climberCurrentPosEntry      = tab.add("Current Pos", -9999)     .withWidget(BuiltInWidgets.kTextView)       .withPosition(9,0).getEntry();
        climberSupplyEntry          = tab.add("Supply Current", -9999)  .withWidget(BuiltInWidgets.kTextView)       .withPosition(8,1).getEntry();
        climberStatorEntry          = tab.add("Stator Current", -9999)  .withWidget(BuiltInWidgets.kTextView)       .withPosition(9,1).getEntry();
        climberBusVoltageEntry      = tab.add("Bus Voltage", -9999)     .withWidget(BuiltInWidgets.kTextView)       .withPosition(8,2).getEntry();
        climberOutputVoltageEntry   = tab.add("Output Voltage", -9999)  .withWidget(BuiltInWidgets.kTextView)       .withPosition(9,2).getEntry();

        lowbarEntry           = tab.add("Low Bar", false)               .withWidget(BuiltInWidgets.kBooleanBox)     .withPosition(3,2).getEntry();
        extendGroundEntry     = tab.add("Extend Ground", false)         .withWidget(BuiltInWidgets.kBooleanBox)     .withPosition(4,2).getEntry();
        slowDriveEntry        = tab.add("Slow Drive", false)            .withWidget(BuiltInWidgets.kBooleanBox)     .withPosition(5,2).getEntry();
        retractExtendEntry    = tab.add("Retract|Extend", false)        .withWidget(BuiltInWidgets.kBooleanBox)     .withPosition(6,2).getEntry();
        intakeEntry           = tab.add("Intake", false)                .withWidget(BuiltInWidgets.kBooleanBox)     .withPosition(3,3).withSize(4,1).getEntry();
    }

    @Override
    public void Enabled()
    {
        LeftMotor.configForwardSoftLimitEnable(true);
        LeftMotor.configReverseSoftLimitEnable(true);

        TestClimberCommand cmd = climber.getClimberCommand();
        if(cmd == null) {cmd = TestClimberCommand.fromState(ClimberState.DEFENSE);} //If null default to defense

        switch(cmd.getStateChange())
        {
            case -1:                if(prevStateChange != -1)                   {prevState();}  break;
            case 0:                 setState(cmd.getState());                                   break;
            case 1:                 if(prevStateChange != 1)                    {nextState();}  break;
            case Integer.MIN_VALUE: if(prevStateChange != Integer.MIN_VALUE)    {resetState();} break;
            default:    break;
        }

        prevStateChange = cmd.getStateChange();

        if(autoCalibrate && !calibrated) setState(ClimberState.CALIBRATING);

        if(climberStatus != ClimberState.CALIBRATING) calibrationPaused = false;
        
        switch(climberStatus)
        {
            case DEFENSE:
                intakeCmd = null;
                LeftMotor.set(TalonFXControlMode.PercentOutput, kDefensePower);
            break;
            case LOW_BAR:
                intakeCmd = TestIntakeCommand.fromState(IntakeState.HARD_STOPS);
            break;
            case EXTEND_GROUND:
                intakeCmd = TestIntakeCommand.fromState(IntakeState.HARD_STOPS);
                LeftMotor.set(TalonFXControlMode.PercentOutput, cmd.getPower());
            break;
            case SLOW_DRIVE:
                intakeCmd = TestIntakeCommand.fromState(IntakeState.HARD_STOPS);
                LeftMotor.set(TalonFXControlMode.PercentOutput,0);
                moveToClimbingMode = false;
            break;
            case RETRACT_EXTEND:
                if (!isAtPos(ClimberPos.RETRACTED,12) && moveToClimbingMode)
                {
                    intakeCmd = TestIntakeCommand.fromState(IntakeState.CLIMBING).setPercentOutput(intakePower);
                }
                else
                {
                    moveToClimbingMode = false;
                    intakeCmd = TestIntakeCommand.fromState(IntakeState.HARD_STOPS);
                }
                LeftMotor.set(TalonFXControlMode.PercentOutput, cmd.getPower());
            break;
            case INTAKE:
                moveToClimbingMode = true;
                intakePower = cmd.getPower() * kIntakeMaxPercent;
                intakeCmd = TestIntakeCommand.fromState(IntakeState.CLIMBING).setPercentOutput(intakePower);
                LeftMotor.set(TalonFXControlMode.PercentOutput,0);
            break;
            case CALIBRATING:
                intakeCmd = null;
                calibrated = false;  
                LeftMotor.configReverseSoftLimitEnable(false);
                if (LeftMotor.getStatorCurrent() > kCalibratingThreshold)
                    calibrationPaused = true;
                if (!calibrationPaused)
                    LeftMotor.set(TalonFXControlMode.PercentOutput, kCalibratingPercent);
                else
                    LeftMotor.set(TalonFXControlMode.PercentOutput, 0);
                if (!CalibrationHallEffect.get()) //Inverted because Hall Effect is stupid
                {
                    LeftMotor.setSelectedSensorPosition(inchesToEncoderUnits(ClimberPos.CALIBRATION.distIn));
                    LeftMotor.set(TalonFXControlMode.PercentOutput, 0);
                    resetState();
                    calibrated = true;
                }  
            break;
            case CUSTOM:
                intakeCmd = null;
                LeftMotor.set(TalonFXControlMode.PercentOutput, cmd.getPower());
            break;
        }
    }

    private double disabledTime = 0;
    @Override
    public void Disabled()
    {
        if(status.Enabled.IsInitState) {disabledTime = Timer.getFPGATimestamp();}
        if(Timer.getFPGATimestamp() - disabledTime > kDisableRecalTimeThreshold) calibrated = false;
        calibrationPaused = false;
    }

    @Override
    public void Update()
    {
        climber = TestClimber.getInstance();

        //Calibration Button
        if(calibrateButton.getBoolean(false))
        {
            calibrated = false;
            calibrateButton.setBoolean(false);
        }
        
        //Updating Status
        status.setIntakeCommand(intakeCmd);

        status.setStatus(climberStatus);
        status.setStatusHistory(ClimberStatusHistory);
        status.setCalibrated(calibrated);
        status.setCalibrationPaused(calibrationPaused);

        status.setStatorCurrent(LeftMotor.getStatorCurrent());
        status.setOutputVoltage(LeftMotor.getMotorOutputVoltage());
        status.setBusVoltage(LeftMotor.getBusVoltage());
        status.setSupplyCurrent(LeftMotor.getSupplyCurrent());
        status.setCurrentPos(encoderUnitsToInches(LeftMotor.getSelectedSensorPosition()));
        status.setIntakeInput(intakePower);

        //Updating Shuffleboard
        statusEntry.setString(status.getStatus().name()); 
        historyEntry.setString(status.getStatusHistory().toString());
        calibratedEntry.setBoolean(status.getCalibrated());
        pauseEntry.setBoolean(status.getCalibrationPaused());
        
        climberStatorEntry.setNumber(status.getStatorCurrent());
        climberOutputVoltageEntry.setNumber(status.getOutputVoltage());
        climberBusVoltageEntry.setNumber(status.getBusVoltage());
        climberSupplyEntry.setNumber(status.getSupplyCurrent());
        climberCurrentPosEntry.setNumber(status.getCurrentPos());
        climbingIntakeInput.setNumber(status.getIntakeInput());
        
        lowbarEntry.setBoolean(false);
        extendGroundEntry.setBoolean(false);
        slowDriveEntry.setBoolean(false);
        retractExtendEntry.setBoolean(false);
        intakeEntry.setBoolean(false);
        switch(climberStatus)
        {
            case INTAKE:
                intakeEntry.setBoolean(true);
            case RETRACT_EXTEND:
                retractExtendEntry.setBoolean(true);
            case SLOW_DRIVE:
                slowDriveEntry.setBoolean(true);
            case EXTEND_GROUND:
                extendGroundEntry.setBoolean(true);
            case LOW_BAR:
                lowbarEntry.setBoolean(true);
            default: break;
        }
    }
/**@deprecated */
    public ClimberState getClimberStatus() {
        try 
        {
            return ClimberStatusHistory.get(ClimberStatusHistory.size()-1);
        }
        catch (IndexOutOfBoundsException o)
        {
            return null;
        }
    }
    public void nextState()
    {
        calibrationPaused = false;
        setState(climberStatus.nextState());
    }
    public void prevState()
    {
        try
        {
            ClimberStatusHistory.remove(ClimberStatusHistory.size()-1);
            climberStatus = ClimberStatusHistory.get(ClimberStatusHistory.size()-1);
        }
        catch (IndexOutOfBoundsException exception)
        {
            resetState();
        }
    }

    public void resetState()
    {
        ClimberStatusHistory.clear();
        ClimberStatusHistory.add(ClimberState.DEFENSE);
        climberStatus = ClimberState.DEFENSE;
    }

    public void setState(ClimberState newState)
    {
        if (climberStatus != newState && newState != null)
        {
            ClimberStatusHistory.add(newState);
            climberStatus = newState;
        }
    }
}
