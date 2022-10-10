package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class TemplateLoop extends LoopBase {
    private static TemplateLoop instance;
    public static TemplateLoop getInstance() {if(instance == null){instance = new TemplateLoop();}return instance;}

    private TemplateStatus status;

    private TemplateLoop()
    {
        //MUST set _status here to either a subclass of StatusBase or to StatusBase itself as a default
        status = TemplateStatus.getInstance();
        _status = status;
        //Shuffleboard stuff goes here, MUST set enabledEntry to a NetworkTableEntry
        tab = Shuffleboard.getTab("Template");
        enabledEntry = tab.add("Enable", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    }

    @Override
    public void Enabled()
    {
        //All code that should run when the system is enabled goes here
        //This should be for actuating motors and other objects on the robot
    }

    @Override
    public void Disabled()
    {
        //All code that should run when the system is disabled goes here
        //This should be for stopping objects and reseting variables
    }

    @Override
    public void Update()
    {
        //All code that should run regardless of the system status goes here
        //Updating Shuffleboard should go here
    }
}
