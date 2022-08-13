package frc.robot.subsystems;

public class ExampleStatus extends Status {
    private static ExampleStatus instance = null;
    public static ExampleStatus getInstance() {if(instance == null){instance = new ExampleStatus();}return instance;}
}
