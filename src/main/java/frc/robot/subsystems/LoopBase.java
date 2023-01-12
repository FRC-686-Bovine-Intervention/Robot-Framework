package frc.robot.subsystems;

import frc.robot.subsystems.StatusBase.EnabledState;

public abstract class LoopBase {
    //TODO: Update documentation
    /**
     * <h3>MUST BE SET IN SUBCLASSES</h3><p>
     * Allows the default enabled status code to run<p>
     * Recommended that you set it to a subclass of {@link SubsystemBase}, but can be set to {@link SubsystemBase} as a default
     */
    protected SubsystemBase Subsystem;
    /**
     * <h3>WARNING: DO NOT OVERRIDE UNLESS YOU KNOW WHAT YOU'RE DOING</h3><p>
     * <h4>Called by {@link SubsystemController} when the robot starts</h4><p>
     */
    public void onStart()
    {
        checkForSubsystem();
        Subsystem.status.Enabled = EnabledState.Starting;
        onEverything();
    }
    /**
     * <h3>WARNING: DO NOT OVERRIDE UNLESS YOU KNOW WHAT YOU'RE DOING</h3><p>
     * <h4>Called by {@link SubsystemController} every loop tick</h4><p>
     */
    public void onLoop()
    {
        onEverything();
    }
    /**
     * <h3>WARNING: DO NOT OVERRIDE UNLESS YOU KNOW WHAT YOU'RE DOING</h3><p>
     * <h4>Called by {@link SubsystemController} when the robot stops</h4><p>
     */
    public void onStop()
    {
        checkForSubsystem();
        Subsystem.status.Enabled = EnabledState.Stopping;
        onEverything();
    }

    private void onEverything()
    {
        checkForSubsystem();
        Update();
        switch(Subsystem.status.Enabled)
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

    private void checkForSubsystem()
    {
        if(Subsystem == null)
            throw new NullPointerException(this.getClass().getName() + " has not defined the super variable _Subsystem\n");
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
