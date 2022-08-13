package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Status.EnabledState;

public abstract class Loop {
    protected Status status;
    protected ShuffleboardTab tab;
    protected NetworkTableEntry enabledEntry;

    public void onLoop()
    {
        switch(status.Enabled)
        {
            case Starting:
                status.Enabled = EnabledState.Enabled;
            break;
            case Enabled:
                if(!enabledEntry.getBoolean(true))
                {
                    status.Enabled = EnabledState.Stopping;
                }
            break;
            case Stopping:
                status.Enabled = EnabledState.Disabled;
            break;
            case Disabled:
                if(enabledEntry.getBoolean(true))
                {
                    status.Enabled = EnabledState.Starting;
                }
            break;
        }
        switch(status.Enabled)
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
        Update();
    }
    public abstract void Enabled();
    public abstract void Disabled();
    public abstract void Update();
}
