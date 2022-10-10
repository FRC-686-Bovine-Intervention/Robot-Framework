package frc.robot.subsystems;

public class TemplateSystem extends SubsystemBase {
    private static TemplateSystem instance;
    public static TemplateSystem getInstance() {if(instance == null){instance = new TemplateSystem();}return instance;}

    private TemplateSystem()
    {
        loop = TemplateLoop.getInstance();
        status = TemplateStatus.getInstance();
    }

    //Extra code, such as commands
}
