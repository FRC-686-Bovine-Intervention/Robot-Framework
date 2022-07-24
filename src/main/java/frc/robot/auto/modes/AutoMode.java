package frc.robot.auto.modes;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.Action;
import frc.robot.lib.util.Pose;

public abstract class AutoMode {
    protected double updatePeriod = Constants.kLoopDt;
    protected boolean active = false;
    protected Pose initialPose = new Pose();
    
    protected abstract void routine() throws AutoModeEndedException;

    public void run() 
    {
        active = true;
        try 
        {
            routine();
        } 
        catch (AutoModeEndedException e) 
        {
            System.out.println("Auto mode done, ended early");
            return;
        }
        done();
        System.out.println("Auto mode done");
    }

    public void done() 
    {
        active = false;
    }

    public void stop() 
    {
        active = false;
    }

    public boolean isActive() 
    {
        return active;
    }

    public boolean isActiveWithThrow() throws AutoModeEndedException 
    {
        if (!isActive()) 
        {
            throw new AutoModeEndedException();
        }
        return isActive();
    }

    public void runAction(Action action) throws AutoModeEndedException 
    {
        action.start();
        while (isActiveWithThrow() && !action.isFinished()) 
        {
        	double currTime = Timer.getFPGATimestamp();
        	double nextTime = Timer.getFPGATimestamp() + updatePeriod;
        	
            action.run();

        	currTime = Timer.getFPGATimestamp();
            long waitTime = (long) ((nextTime-currTime) * 1000.0);	// attempt to run thread every updatePeriod seconds
            waitTime = Math.max(waitTime, 0);						// avoid negative waits
            try
            {
                Thread.sleep(waitTime);
            } 
            catch (InterruptedException e) 
            {
                e.printStackTrace();
            }
        }
        action.done();
    }

    public Pose getInitialPose()
    {
    	return initialPose;	// default implementation
    }
}
