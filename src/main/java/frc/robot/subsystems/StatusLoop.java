package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.loops.Loop;

public abstract class StatusLoop implements Loop{

    protected ShuffleboardTab tab = Shuffleboard.getTab("Example");
    protected NetworkTableEntry enabledEntry = tab.add("Enable", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    @Override
    public void onLoop()
    {

    }
    public abstract void run();
}
