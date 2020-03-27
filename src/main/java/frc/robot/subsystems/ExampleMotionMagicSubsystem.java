package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.drivers.TalonSRXChecker;
import frc.robot.lib.util.Util;

/**
 * This class is an example framework for a subsystem that uses TalonSRX Motion
 * Magic (Elevators, Arms, Wrists, etc.)
 */

public class ExampleMotionMagicSubsystem extends Subsystem
{
    private static ExampleMotionMagicSubsystem mInstance = null;

    private Command command = new Command();
    private Status  status  = new Status();

    public TalonSRX master;
    public TalonSRX slave;

    public boolean zeroed = false; // set when we hit the zero limit switch and set our position

    // Min/Max motor outputs (Talon Software Manual Section 10.5)
    private double kNeutralDeadband = 0.001; // deadband where motor output will be set to neutral. range is (0.001 to 0.25,
                                             // which is 0.1% to 25%)
    private double kMinOutput = 0.0; // minimum non-zero motor outpu. increase to overcome stiction, etc.
    private double kMaxOutput = 1.0; // maximum motor output

    // limit switch limits
    public final boolean limitSwitchConnectedToTalon = false; // true: use limit switch connected to Talon false: use limit switch connected
                                                              // to roboRIO
    public DigitalInput limitSwitch;
    private final double kMinLimit = 0.0; // inches
    private final double kMaxLimit = 10.0; // inches

    // encoder constants
    private static final double kEncoderGain = 1.0; // number of drive shaft rotations per encoder shaft rotation
    // private static final FeedbackDevice kEncoderType =
    // FeedbackDevice.QuadEncoder;
    // private static final int kEncoderCodesPerRev = 64; // Grayhill encoder has 64
    // native units per revolution.
    // private static final FeedbackDevice kEncoderType =
    // FeedbackDevice.QuadEncoder;
    // private static final int kEncoderCodesPerRev = 256; // Grayhill encoder has
    // 256 native units per revolution.
    private static final FeedbackDevice kEncoderType = FeedbackDevice.CTRE_MagEncoder_Relative;
    private static final int kEncoderCodesPerRev = 4096; // CTRE Mag encoder has 4096 native units per revolution.
    private static final int kEncoderUnitsPerRev = (int) (4 * kEncoderCodesPerRev / kEncoderGain);
    private static final double kEncoderStatusFramePeriod = 0.100; // 100 ms
    private static final double kWheelCircumInches = 4.0;           // circumference of wheel/sprocket

    // Ramp Rates (Talon Software Manual Section 17.1.4)
    private double kOpenLoopRampRate = 0.1; // seconds from 0 to full output
    private double kClosedLoopRampRate = 0.1; // seconds from 0 to full output

    // Current Limits (Talon Software Manual Section 9.3)
    private int kContinuousCurrentLimit = 20; // amps
    private int kPeakCurrentLimit = 35; // amps
    private int kPeakCurrentDuration = 200; // ms

    // Closed-Loop Parameters

    // calibration measurements used to set closed-loop gains
    // (measured using NI web interface)
    private double kCalibEncoderPulsePer100ms = 999; // calibration velocity at a max throttle
    private double kCalibPercentOutput = 1.0; // percent output of master at above calibration throttle

    // Position Closed-Loop PID gains (Talon Software Manual Section 10.1)
    // Units: setpoint, error, and output are in native units per 100ms.
    private int kPositionSlotIdx = 0;
    private double kPositionKf = kCalibPercentOutput * 1023.0 / kCalibEncoderPulsePer100ms;
    private double kPositionKp = 0.4;
    private double kPositionKd = 0.0; // to resolve any overshoot, start at 10*Kp
    private double kPositionKi = 0.0;
    private int kPositionIZone = 0;

    // set Allowable Closed-Loop Error (Talon Software Manual Section 10.6)
    private final int kPositionAllowableError = 0; // when error is less than this, P,I&D terms are zeroed, integral accumulator is
                                                   // cleared (default 0)

    // Motion Magic (Talon Software Manual Section 12.6.5)
    private final double kTimeToCruiseVelocity = 0.25; // seconds to reach cruise velocity
    private final int kCruiseVelocity = (int) (0.75 * kCalibEncoderPulsePer100ms); // cruise below top speed
    private final int kAccel = (int) (kCruiseVelocity / kTimeToCruiseVelocity);

    private final double kManualPercentOutput = 0.25;

    public synchronized static ExampleMotionMagicSubsystem getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new ExampleMotionMagicSubsystem();
        }
        return mInstance;
    }

    private ExampleMotionMagicSubsystem()
    {
        master = new TalonSRX(Constants.kExampleMasterDeviceId);
        slave = new TalonSRX(Constants.kExampleSlaveDeviceId);

        // Configure Remote Sensor / Encoder (Talon Software Manual Section 9.8.3)
        master.configSelectedFeedbackSensor(kEncoderType, Constants.kTalonPidIdx, Constants.kLongCANTimeoutMs);
        master.setSensorPhase(true);
        master.setInverted(false);

        // set Peak/Nominal Output Limits (Talon Software Manual Section 10.5)
        master.configNominalOutputForward(+kMinOutput, Constants.kLongCANTimeoutMs);
        master.configNominalOutputReverse(-kMinOutput, Constants.kLongCANTimeoutMs);
        master.configPeakOutputForward(+kMaxOutput, Constants.kLongCANTimeoutMs);
        master.configPeakOutputReverse(-kMaxOutput, Constants.kLongCANTimeoutMs);
        master.configNeutralDeadband(kNeutralDeadband, Constants.kLongCANTimeoutMs);

        // enable compensation for battery voltage droop (Talon Software Manual Section
        // 9.2)
        master.configVoltageCompSaturation(Constants.kNominalBatteryVoltage, Constants.kLongCANTimeoutMs);
        master.enableVoltageCompensation(true);

        // set ramp rates / current limits (Talon Software Manual Section 9.3)
        master.configClosedloopRamp(kClosedLoopRampRate, Constants.kLongCANTimeoutMs);
        master.configOpenloopRamp(kOpenLoopRampRate, Constants.kLongCANTimeoutMs);
        master.configContinuousCurrentLimit(kContinuousCurrentLimit, Constants.kLongCANTimeoutMs);
        master.configPeakCurrentLimit(kPeakCurrentLimit, Constants.kLongCANTimeoutMs);
        master.configPeakCurrentDuration(kPeakCurrentDuration, Constants.kLongCANTimeoutMs);
        master.enableCurrentLimit(true);

        // set relevant frame periods to be at least as fast as periodic rate (Talon
        // Software Manual Section 12.6, 20.8.2)
        master.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, (int) (1000 * Constants.kLoopDt), Constants.kLongCANTimeoutMs);
        master.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, (int) (1000 * Constants.kLoopDt), Constants.kLongCANTimeoutMs);
        master.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, (int) (1000 * Constants.kLoopDt), Constants.kLongCANTimeoutMs);

        // Configure Position Closed-Loop Control PID (Talon Software Manual Section
        // 10.1)
        // including Allowable Closed-Loop Error (Talon Software Manual Section 10.6)
        master.selectProfileSlot(kPositionSlotIdx, Constants.kTalonPidIdx);
        master.config_kF(kPositionSlotIdx, kPositionKf, Constants.kLongCANTimeoutMs);
        master.config_kP(kPositionSlotIdx, kPositionKp, Constants.kLongCANTimeoutMs);
        master.config_kI(kPositionSlotIdx, kPositionKi, Constants.kLongCANTimeoutMs);
        master.config_kD(kPositionSlotIdx, kPositionKd, Constants.kLongCANTimeoutMs);
        master.config_IntegralZone(kPositionSlotIdx, kPositionIZone, Constants.kLongCANTimeoutMs);
        master.configAllowableClosedloopError(kPositionSlotIdx, kPositionAllowableError, Constants.kLongCANTimeoutMs);

        // set Motion Magic Cruise Velocity/Acceleration (Talon Software Manual Section
        // 12.6.5)
        master.configMotionCruiseVelocity(kCruiseVelocity, Constants.kLongCANTimeoutMs);
        master.configMotionAcceleration(kAccel, Constants.kLongCANTimeoutMs);

        // set soft limits
        enableSoftLimits();

        // limit switch
        if (limitSwitchConnectedToTalon)
        {
            master.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, Constants.kLongCANTimeoutMs);
        }
        else
        {
            limitSwitch = new DigitalInput(Constants.kExampleSubsystemLimitSwitchPwmId);
        }

        // configure slave motor controllers to follow master
        slave.follow(master);

        // initial state of motors is stopped, waiting to be zeroed
        zeroed = false;
        stop();
    }

    public void enableSoftLimits()
    {
        master.configReverseSoftLimitThreshold(inchesToEncoderUnits(kMinLimit), Constants.kCANTimeoutMs);
        master.configForwardSoftLimitThreshold(inchesToEncoderUnits(kMaxLimit), Constants.kCANTimeoutMs);
        master.configReverseSoftLimitEnable(true, Constants.kCANTimeoutMs);
        master.configForwardSoftLimitEnable(true, Constants.kCANTimeoutMs);
        master.overrideLimitSwitchesEnable(true);
    }

    public void disableSoftLimits()
    {
        master.configReverseSoftLimitEnable(false, Constants.kCANTimeoutMs);
        master.configForwardSoftLimitEnable(false, Constants.kCANTimeoutMs);
        master.overrideLimitSwitchesEnable(false); // disable soft limit switches
    }

    @Override
    public void stop()
    {
        // write new command to stop
        command.setMotorPercentOutput(0.0);
        command.setNeutralMode(NeutralMode.Brake); 

        // also write immediately to motors in case Subsystem loop is not working
        master.set(ControlMode.PercentOutput, 0.0);
        master.setNeutralMode(NeutralMode.Brake);
        slave.setNeutralMode(NeutralMode.Brake);
    }

    public void manualUp()
    {
        command.setMotorPercentOutput(+kManualPercentOutput);
    }
    
    public void manualDown()
    {
        command.setMotorPercentOutput(-kManualPercentOutput);
    }
    
    public void manualStop()
    {
        command.setMotorPercentOutput(0.0);
    }
    
    public synchronized void setTarget(double targetPositionInches)
    {
        command.setTargetPositionInches(targetPositionInches);
    }
    
    public synchronized boolean atTarget()
    {
        boolean rv = false;
        double toleranceInches = 1.0;
        if ((status.controlMode == ControlMode.MotionMagic)
        && Util.epsilonEquals(status.getTrajectoryPositionInches(), status.getTrajectoryTargetInches(), toleranceInches))
        rv = true;
        return rv;
    }
    
    @Override
    public synchronized void zeroSensors()
    {
        master.setSelectedSensorPosition(0, Constants.kTalonPidIdx, Constants.kCANTimeoutMs);
        zeroed = true;
    }

    public synchronized boolean hasBeenZeroed()
    {
        return zeroed;
    }

    public synchronized void zeroIfAtLimit()
    {
        if (status.isLimitSwitchTriggered())
        {
            zeroSensors();
        }
    }

    @Override
    public synchronized void sendCommands()
    {
        if (command.getControlMode() == ControlMode.PercentOutput)
        {
            master.set(ControlMode.MotionMagic, inchesToEncoderUnits(command.getDemand()), DemandType.ArbitraryFeedForward, command.getFeedForward());
        }
        else if (command.controlMode == ControlMode.MotionMagic)
        {
            master.set(ControlMode.PercentOutput, command.getDemand(), DemandType.ArbitraryFeedForward, command.getFeedForward());
        }
        else
        {
            System.out.println("Unknown command control mode: " + command.getControlMode().toString());
        }

        master.setNeutralMode(command.neutralMode);
         slave.setNeutralMode(command.neutralMode);
    }

    @Override
    public synchronized void getStatus()
    {
        status.time = Timer.getFPGATimestamp();

        // read status once per update
        status.setPositionInches( encoderUnitsToInches(master.getSelectedSensorPosition(Constants.kTalonPidIdx)) );
        status.setVelocityInchesPerSec( encoderUnitsPerFrameToInchesPerSecond(master.getSelectedSensorVelocity(Constants.kTalonPidIdx)) );

        if (master.getControlMode() == ControlMode.MotionMagic)
        {
            status.setPidError( master.getClosedLoopError(Constants.kTalonPidIdx) );
            status.setTrajectoryTargetInches( encoderUnitsToInches(master.getClosedLoopTarget(Constants.kTalonPidIdx)) );
            status.setTrajectoryPositionInches( encoderUnitsToInches(master.getActiveTrajectoryPosition()) );
            status.setTrajectoryVelocityInchesPerSec( encoderUnitsPerFrameToInchesPerSecond(master.getActiveTrajectoryVelocity()) );
        }    
        else
        {
            status.setPidError(0.0);
            status.setTrajectoryTargetInches(Integer.MIN_VALUE);
            status.setTrajectoryPositionInches(Integer.MIN_VALUE);
            status.setTrajectoryVelocityInchesPerSec(0.0);
        }    

        status.setMotorPercentOutput( master.getMotorOutputPercent() );
        status.setMotorCurrent( master.getOutputCurrent() );

        if (limitSwitchConnectedToTalon)
        {
            status.setLimitSwitchTriggered( master.getSensorCollection().isFwdLimitSwitchClosed() );
        }    
        else
        {
            status.setLimitSwitchTriggered( !limitSwitch.get() );
        }    
    }    

    @Override
    public boolean checkSystem()
    {
        // TODO: add checkSystem
        return true;
    }

    // Talon SRX reports position in rotations while in closed-loop Position mode
    private static double encoderUnitsToInches(int _encoderPosition)
    {
        return (double) _encoderPosition / (double) kEncoderUnitsPerRev * kWheelCircumInches;
    }

    private static int inchesToEncoderUnits(double _inches)
    {
        return (int) (_inches / kWheelCircumInches * kEncoderUnitsPerRev);
    }

    // Talon SRX reports speed in RPM while in closed-loop Speed mode
    private static double encoderUnitsPerFrameToInchesPerSecond(int _encoderEdgesPerFrame)
    {
        return encoderUnitsToInches(_encoderEdgesPerFrame) / kEncoderStatusFramePeriod;
    }

    private static int inchesPerSecondToEncoderUnitsPerFrame(double _inchesPerSecond)
    {
        return (int) (inchesToEncoderUnits(_inchesPerSecond) * kEncoderStatusFramePeriod);
    }

    public class Command
    {
        public ControlMode controlMode = ControlMode.PercentOutput;
        public NeutralMode neutralMode = NeutralMode.Coast;
        public double demand = 0.0;
        public double feedForward = 0.0;
        
        public Command() {}
    
        public synchronized void setMotorPercentOutput(double motorPercentOutput)
        {
            this.controlMode = ControlMode.PercentOutput;
            this.demand = motorPercentOutput;
        }

        public synchronized void setTargetPositionInches(double targetPositionInches)
        {
            this.controlMode = ControlMode.MotionMagic;
            this.demand = targetPositionInches; 
        }

        public synchronized void setNeutralMode(NeutralMode neutralMode)
        {
            this.neutralMode = neutralMode;
        }

       // @formatter:off
       public synchronized ControlMode getControlMode() { return controlMode; }
       public synchronized NeutralMode getNeutralMode() { return neutralMode; }
       public synchronized double getDemand() { return demand; }
       public synchronized double getFeedForward() { return feedForward; }
       // @formatter:on
    }
    
    public class Status
    {
        public double time = 0.0;
        
        public ControlMode controlMode = ControlMode.PercentOutput;

        public double positionInches = 0.0;
        public double velocityInchesPerSec = 0.0;

        public double trajectoryTargetInches = 0.0;
        public double trajectoryPositionInches = 0.0;
        public double trajectoryVelocityInchesPerSec = 0.0;

        public double pidError = 0.0;
        public double motorPercentOutput = 0.0;
        public double motorCurrent = 0.0;

        public boolean limitSwitchTriggered = false;

        // @formatter:off
        public Status() {}

        public synchronized void setPositionInches(double positionInches) { this.positionInches = positionInches; }
        public synchronized void setVelocityInchesPerSec(double velocityInchesPerSec) { this.velocityInchesPerSec = velocityInchesPerSec; }
        public synchronized void setTrajectoryTargetInches(double trajectoryTargetInches) { this.trajectoryTargetInches = trajectoryTargetInches; }
        public synchronized void setTrajectoryPositionInches(double trajectoryPositionInches) { this.trajectoryPositionInches = trajectoryPositionInches; }
        public synchronized void setTrajectoryVelocityInchesPerSec(double trajectoryVelocityInchesPerSec) { this.trajectoryVelocityInchesPerSec = trajectoryVelocityInchesPerSec; }
        public synchronized void setPidError(double pidError) { this.pidError = pidError; }
        public synchronized void setMotorPercentOutput(double motorPercentOutput) { this.motorPercentOutput = motorPercentOutput; }
        public synchronized void setMotorCurrent(double motorCurrent) { this.motorCurrent = motorCurrent; }
        public synchronized void setLimitSwitchTriggered(boolean limitSwitchTriggered) { this.limitSwitchTriggered = limitSwitchTriggered; }

        public synchronized double getPositionInches() { return positionInches; }
        public synchronized double getVelocityInchesPerSec() { return velocityInchesPerSec; }
        public synchronized double getTrajectoryTargetInches() { return trajectoryTargetInches; }
        public synchronized double getTrajectoryPositionInches() { return trajectoryPositionInches; }
        public synchronized double getTrajectoryVelocityInchesPerSec() { return trajectoryVelocityInchesPerSec; }
        public synchronized double getPidError() { return pidError; }
        public synchronized double getMotorPercentOutput() { return motorPercentOutput; }
        public synchronized double getMotorCurrent() { return motorCurrent; }
        public synchronized boolean isLimitSwitchTriggered() { return limitSwitchTriggered; }
        // @formatter:on

    }

    @Override
    public void outputTelemetry()
    {
        synchronized (status)
        {
            SmartDashboard.putNumber( "ExampleSubsystem/positionInches", status.getPositionInches() );
            SmartDashboard.putNumber( "ExampleSubsystem/velocityInchesPerSec", status.getVelocityInchesPerSec() );
            SmartDashboard.putNumber( "ExampleSubsystem/trajectoryTargetInches", status.getTrajectoryTargetInches() );
            SmartDashboard.putNumber( "ExampleSubsystem/trajectoryPositionInches", status.getTrajectoryPositionInches() );
            SmartDashboard.putNumber( "ExampleSubsystem/trajectoryVelocityInchesPerSec", status.getTrajectoryVelocityInchesPerSec() );
            SmartDashboard.putNumber( "ExampleSubsystem/pidError", status.getPidError() );
            SmartDashboard.putNumber( "ExampleSubsystem/motorPercentOutput", status.getMotorPercentOutput() );
            SmartDashboard.putNumber( "ExampleSubsystem/motorCurrent",  status.getMotorCurrent() );
            SmartDashboard.putBoolean("ExampleSubsystem/limitSwitch",  status.isLimitSwitchTriggered() );
        }
    }
}
