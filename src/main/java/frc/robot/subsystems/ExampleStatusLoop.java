package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.loops.Loop;
import frc.robot.subsystems.Status.EnabledState;

public class ExampleStatusLoop implements Loop{
    private ExampleStatus status = ExampleStatus.getInstance();
    private ShuffleboardTab tab = Shuffleboard.getTab("Example");
    private NetworkTableEntry enabledEntry = tab.add("Enable", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();

    @Override
    public void onStart() {
        
    }

    @Override
    public void onLoop() {
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
    }

    @Override
    public void onStop() {
        
    }
}
