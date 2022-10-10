package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.StatusBase.EnabledState;

public abstract class LoopBase {
    /**
     * <h3>MUST BE SET IN SUBCLASSES</h3><p>
     * Allows the default enabled status code to run<p>
     * Recommended that you set it to a subclass of {@link StatusBase}, but can be set to {@link StatusBase} as a default
     */
    protected StatusBase _status;
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
        if (_status == null) {throw new NullPointerException(this.getClass().getName() + " has not defined the super variable _status\n");}
        _status.Enabled = EnabledState.Starting;
        onEverything();
    }
    /**
     * <h3>WARNING: DO NOT OVERRIDE UNLESS YOU KNOW WHAT YOU'RE DOING</h3><p>
     * <h4>Called by {@link SubsystemController} every loop tick</h4><p>
     */
    public void onLoop()
    {
        if (_status == null) {throw new NullPointerException(this.getClass().getName() + " has not defined the super variable _status\n");}
        if(enabledEntry == null)
        {
            switch(_status.Enabled)
            {
                case Starting:
                    _status.Enabled = EnabledState.Enabled;
                break;
                case Enabled:
                    if(DriverStation.isDisabled())
                    {
                        _status.Enabled = EnabledState.Stopping;
                    }
                break;
                case Stopping:
                    _status.Enabled = EnabledState.Disabled;
                break;
                case Disabled:
                    if(DriverStation.isEnabled())
                    {
                        _status.Enabled = EnabledState.Starting;
                    }
                break;
            }
        }
        else
        {
            switch(_status.Enabled)
            {
                case Starting:
                    _status.Enabled = EnabledState.Enabled;
                break;
                case Enabled:
                    if(DriverStation.isDisabled() || !enabledEntry.getBoolean(true))
                    {
                        _status.Enabled = EnabledState.Stopping;
                    }
                break;
                case Stopping:
                    _status.Enabled = EnabledState.Disabled;
                break;
                case Disabled:
                    if(DriverStation.isEnabled() && enabledEntry.getBoolean(true))
                    {
                        _status.Enabled = EnabledState.Starting;
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
        _status.Enabled = EnabledState.Stopping;
        onEverything();
    }

    private void onEverything()
    {
        Update();
        switch(_status.Enabled)
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
}
