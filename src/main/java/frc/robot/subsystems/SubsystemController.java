package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.util.CrashTrackingRunnable;

public class SubsystemController
{
    private static SubsystemController instance;
    public static SubsystemController getInstance() {if(instance == null){instance = new SubsystemController();}return instance;} 
    public final double kPeriod = 0.01;

    private boolean running_;

    private final Notifier notifier_;	// the Notifier will run the function runCrashTracked() with a period of kPeriod
    
    private final List<SubsystemBase> subsystems_;
    private final Object taskRunningLock_ = new Object();
    private double prev_time_ = 0;
	protected double dt_;
   
    private final CrashTrackingRunnable runnable_ = new CrashTrackingRunnable() 
    {
        @Override
        public void runCrashTracked() 
        {
        	// lock during access to loop_ to avoid corruption from multiple threads
            synchronized (taskRunningLock_) 
            {
                if (running_) 
                {
                    double curr_time = Timer.getFPGATimestamp();
                    for (SubsystemBase subs : subsystems_) 
                    {
                        subs.loop.onLoop();
                    }
                    dt_ = curr_time - prev_time_;
                    prev_time_ = curr_time;
                }
            }
        }
    };
    
    public SubsystemController() 
    {
        notifier_ = new Notifier(runnable_);
        running_ = false;
        subsystems_ = new ArrayList<>();
    }

    public synchronized void register(SubsystemBase subsystem) 
    {
    	// lock during access to loop_ to avoid corruption from multiple threads
        synchronized (taskRunningLock_) 
        {
            subsystems_.add(subsystem);
        }
    }

    public synchronized void start() 
    {
        if (!running_) 
        {
            System.out.println("Starting loops");
        	// lock during access to loop_ to avoid corruption from multiple threads
            synchronized (taskRunningLock_) 
            {
                prev_time_ = Timer.getFPGATimestamp();
                for (SubsystemBase subs : subsystems_) 
                {
//                    System.out.println("Starting " + loop);
                    subs.loop.onStart();
                }
                running_ = true;
            }
            // avoiding watchdog errors
            notifier_.startPeriodic(kPeriod);
        }
    }

    public synchronized void stop() 
    {
        if (running_) 
        {
            System.out.println("Stopping loops");
            notifier_.stop();
        	// lock during access to loop_ to avoid corruption from multiple threads
            synchronized (taskRunningLock_) 
            {
                running_ = false;
                for (SubsystemBase subs : subsystems_) 
                {
//                    System.out.println("Stopping " + loop);
                    subs.loop.onStop();
                }
            }
        }
    }

}
