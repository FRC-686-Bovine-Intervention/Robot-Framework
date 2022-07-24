package frc.robot.auto.actions;

public interface Action {
    /**
     * Run code once when the action is started, for set up
     */
    public abstract void start();

    /**
     * Called by runAction in AutoModeBase iteratively until isFinished returns
     * true. Iterative logic lives in this method
     */
    public abstract void run();

    /**
     * Returns whether or not the code has finished execution. When implementing
     * this interface, this method is used by the runAction method every cycle
     * to know when to stop running the action
     * 
     * @return boolean
     */
    public abstract boolean isFinished();

    /**
     * Run code once when the action finishes, usually for clean up
     */
    public abstract void done();
}
