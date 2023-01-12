package frc.robot.subsystems;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import io.github.oblarg.oblog.Loggable;

public class SubsystemController implements Loggable
{
    private static SubsystemController instance;
    public static SubsystemController getInstance() {if(instance == null){instance = new SubsystemController();}return instance;} 
    
    private final ArrayList<SubsystemBase> subsystems;

    private SubsystemController() 
    {
        subsystems = new ArrayList<>();
    }

    public SubsystemController register(SubsystemBase subsystem)
    {
        subsystems.add(subsystem);
        return this;
    }

    public SubsystemController start()
    {
        for (SubsystemBase sub : subsystems)
        {
            sub.init();
            sub.status.inputs();
            Logger.getInstance().processInputs(sub.getClass().getName(), sub.status);
            sub.loop.onStart();
            sub.status.record();
        }
        return this;
    }
    
    public SubsystemController run()
    {
        for (SubsystemBase sub : subsystems)
        {
            sub.status.inputs();
            Logger.getInstance().processInputs(sub.getClass().getSimpleName(), sub.status);
            sub.loop.onLoop();
            sub.status.record();
        }
        return this;
    }

    public SubsystemController stop()
    {
        for (SubsystemBase sub : subsystems)
        {
            sub.status.inputs();
            Logger.getInstance().processInputs(sub.getClass().getName(), sub.status);
            sub.loop.onStop();
            sub.status.record();
        }
        return this;
    }
    //     public final double kPeriod = 0.01;

//     private boolean running_;

//     private final Notifier notifier_;	// the Notifier will run the function runCrashTracked() with a period of kPeriod
    
//     private final Object taskRunningLock_ = new Object();
//     private double prev_time_ = 0;
// 	protected double dt_;
   
//     private final CrashTrackingRunnable runnable_ = new CrashTrackingRunnable() 
//     {
//         @Override
//         public void runCrashTracked() 
//         {
//         	// lock during access to loop_ to avoid corruption from multiple threads
//             synchronized (taskRunningLock_) 
//             {
//                 if (running_) 
//                 {
//                     double curr_time = Timer.getFPGATimestamp();
//                     for (SubsystemBase subs : subsystems_) 
//                     {
//                         subs.loop.onLoop();
//                     }
//                     dt_ = curr_time - prev_time_;
//                     prev_time_ = curr_time;
//                 }
//             }
//         }
//     };
    

//     public synchronized void register(SubsystemBase subsystem) 
//     {
//     	// lock during access to loop_ to avoid corruption from multiple threads
//         synchronized (taskRunningLock_) 
//         {
//             subsystems_.add(subsystem);
//         }
//     }

//     public synchronized void start() 
//     {
//         if (!running_) 
//         {
//             System.out.println("Starting loops");
//         	// lock during access to loop_ to avoid corruption from multiple threads
//             synchronized (taskRunningLock_) 
//             {
//                 prev_time_ = Timer.getFPGATimestamp();
//                 for (SubsystemBase subs : subsystems_) 
//                 {
// //                    System.out.println("Starting " + loop);
//                     subs.loop.onStart();
//                 }
//                 running_ = true;
//             }
//             // avoiding watchdog errors
//             notifier_.startPeriodic(kPeriod);
//         }
//     }

//     public synchronized void stop() 
//     {
//         if (running_) 
//         {
//             System.out.println("Stopping loops");
//             notifier_.stop();
//         	// lock during access to loop_ to avoid corruption from multiple threads
//             synchronized (taskRunningLock_) 
//             {
//                 running_ = false;
//                 for (SubsystemBase subs : subsystems_) 
//                 {
// //                    System.out.println("Stopping " + loop);
//                     subs.loop.onStop();
//                 }
//             }
//         }
//     }

}
