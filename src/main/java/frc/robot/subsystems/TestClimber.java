package frc.robot.subsystems;

public class TestClimber extends SubsystemBase {
    private static TestClimber instance;
    public static TestClimber getInstance() {if(instance == null){instance = new TestClimber();}return instance;}

    private TestClimber()
    {
        loop = TestClimberLoop.getInstance();
        status = TestClimberStatus.getInstance();
    }

    private TestClimberCommand climberCmd;

    public TestClimber setClimberCommand(TestClimberCommand climberCmd) {this.climberCmd = climberCmd; return this;}
    public TestClimberCommand getClimberCommand()                       {return climberCmd;}
}
