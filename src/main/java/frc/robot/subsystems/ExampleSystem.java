package frc.robot.subsystems;

public class ExampleSystem extends Subsystem {
    private ExampleSystem instance;
    public ExampleSystem getInstance() {if(instance == null){instance = new ExampleSystem();}return instance;}

    private ExampleSystem()
    {
        loop = ExampleLoop.getInstance();
        status = ExampleStatus.getInstance();
    }
}
