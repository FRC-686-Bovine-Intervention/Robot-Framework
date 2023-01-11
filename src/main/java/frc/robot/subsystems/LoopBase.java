package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.StatusBase.EnabledState;

public abstract class LoopBase {
    //TODO: Update documentation
    /**
     * <h3>MUST BE SET IN SUBCLASSES</h3><p>
     * Allows the default enabled status code to run<p>
     * Recommended that you set it to a subclass of {@link SubsystemBase}, but can be set to {@link SubsystemBase} as a default
     */
    protected SubsystemBase _Subsystem;
    /**
     * Allows subclasses to have cleaner code by not having to initialize a tab<p>
     * Might get removed
     */
    protected ShuffleboardTab tab;
    /**
     * <h3>Recommended to be set in subclasses</h3><p>
     * Allows the default Shuffleboard enabled switch to affect {@link #_status}
     */
    protected NetworkTableEntry enabledEntry;
    /**
     * <h3>WARNING: DO NOT OVERRIDE UNLESS YOU KNOW WHAT YOU'RE DOING</h3><p>
     * <h4>Called by {@link SubsystemController} when the robot starts</h4><p>
     */
    public void onStart()
    {
        if (_Subsystem == null) {throw new NullPointerException(this.getClass().getName() + " has not defined the super variable _Subsystem\n");}
        _Subsystem.status.Enabled = EnabledState.Starting;
        onEverything();
    }
    /**
     * <h3>WARNING: DO NOT OVERRIDE UNLESS YOU KNOW WHAT YOU'RE DOING</h3><p>
     * <h4>Called by {@link SubsystemController} every loop tick</h4><p>
     */
    public void onLoop()
    {
        if (_Subsystem.status == null) {throw new NullPointerException(this.getClass().getName() + " has not defined the super variable _Subsystem\n");}
        if(enabledEntry == null)
        {
            switch(_Subsystem.status.Enabled)
            {
                case Starting:
                    _Subsystem.status.Enabled = EnabledState.Enabled;
                break;
                case Enabled:
                    if(DriverStation.isDisabled())
                    {
                        _Subsystem.status.Enabled = EnabledState.Stopping;
                    }
                break;
                case Stopping:
                    _Subsystem.status.Enabled = EnabledState.Disabled;
                break;
                case Disabled:
                    if(DriverStation.isEnabled())
                    {
                        _Subsystem.status.Enabled = EnabledState.Starting;
                    }
                break;
            }
        }
        else
        {
            switch(_Subsystem.status.Enabled)
            {
                case Starting:
                    _Subsystem.status.Enabled = EnabledState.Enabled;
                break;
                case Enabled:
                    if(DriverStation.isDisabled() || !enabledEntry.getBoolean(true))
                    {
                        _Subsystem.status.Enabled = EnabledState.Stopping;
                    }
                break;
                case Stopping:
                    _Subsystem.status.Enabled = EnabledState.Disabled;
                break;
                case Disabled:
                    if(DriverStation.isEnabled() && enabledEntry.getBoolean(true))
                    {
                        _Subsystem.status.Enabled = EnabledState.Starting;
                    }
                break;
            }
        }
        onEverything();
    }
    /**
     * <h3>WARNING: DO NOT OVERRIDE UNLESS YOU KNOW WHAT YOU'RE DOING</h3><p>
     * <h4>Called by {@link SubsystemController} when the robot stops</h4><p>
     */
    public void onStop()
    {
        _Subsystem.status.Enabled = EnabledState.Stopping;
        onEverything();
    }

    private void onEverything()
    {
        UpdateStatus();
        Logger.getInstance().processInputs(_Subsystem.getClass().toString(), _Subsystem.status);
        Update();
        switch(_Subsystem.status.Enabled)
        {
            case Starting:
            case Enabled:
                Enabled();
            break;
            case Stopping:
            case Disabled:
                Disabled();
            break;
        }
        _Subsystem.status.recordOutputs();
    }

    /**
     * <h3>Runs every loop tick when the subsystem is enabled</h3><p>
     * All code that should run when the subsystem is enabled goes here<p>
     * This should be for actuating motors and other objects on the robot
     */
    public abstract void Enabled();
    /**
     * <h3>Runs every loop tick when the subsystem is disabled</h3><p>
     * All code that should run when the system is disabled goes here<p>
     * This should be for stopping objects and reseting variables
     */
    public abstract void Disabled();
    /**
     * <h3>Runs every loop tick</h3><p>
     * All code that should run regardless of the system status goes here<p>
     * This should be for things like updating Shuffleboard
     */
    public abstract void Update();
    /**
     * <h3>Runs every loop tick</h3><p>
     * Code here should be updating the status's inputs to be logged to AdvantageKit<p>
     */
    public abstract void UpdateStatus();
}
