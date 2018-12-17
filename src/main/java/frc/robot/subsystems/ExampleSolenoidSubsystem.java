package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;

public class ExampleSolenoidSubsystem extends Subsystem
{
    private static ExampleSolenoidSubsystem mInstance = null;
    private final boolean kSolenoidStateWhenExtended = true;    // negate this to reverse to reverse the controls polarity
    private Solenoid solenoid;
    private boolean direction;
 

    public synchronized static ExampleSolenoidSubsystem getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new ExampleSolenoidSubsystem();
        }
        return mInstance;
    }

    private ExampleSolenoidSubsystem()
    {
        solenoid = new Solenoid(Constants.kExample1ForwardChannel);

        // Start the cylinder in the retracted position. 
        direction = kSolenoidStateWhenExtended; // Set direction to extended to force a state change
        retract();
    }

    // We only call the set() function when the solenoid state changes.
    // This reduces the amount of time required to check the state, and reduces the amount of data on the CAN bus
 
    public synchronized void extend()
    {
        if (direction != kSolenoidStateWhenExtended)
        {
            direction = kSolenoidStateWhenExtended;
            solenoid.set(direction);
         }
    }

    public synchronized void retract()
    {
        if (direction == kSolenoidStateWhenExtended)
        {
            direction = !kSolenoidStateWhenExtended;
            solenoid.set(direction);
         }
    }

    @Override
    public boolean checkSystem()
    {
        return true;
    }

    @Override
    public void outputTelemetry()
    {
    }

    @Override
    public void stop()
    {
    }

    @Override
    public void zeroSensors()
    {
    }
}