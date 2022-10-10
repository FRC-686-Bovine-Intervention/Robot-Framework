package frc.robot.subsystems;

public class TemplateStatus extends StatusBase {
    private static TemplateStatus instance = null;
    public static TemplateStatus getInstance() {if(instance == null){instance = new TemplateStatus();}return instance;}

    //All variables that are part of the overall status of the system go here

    //Setters and getters should be synchronized
}
