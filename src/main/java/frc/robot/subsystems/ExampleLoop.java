package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class ExampleLoop extends Loop {
    private static ExampleLoop instance;
    public static ExampleLoop getInstance() {if(instance == null){instance = new ExampleLoop();}return instance;}

    private ExampleLoop()
    {
        status = ExampleStatus.getInstance();
        tab = Shuffleboard.getTab("Example");
        enabledEntry = tab.add("Enable", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    }

    @Override
    public void Enabled()
    {

    }

    @Override
    public void Disabled()
    {

    }

    @Override
    public void Update()
    {
        
    }
}
