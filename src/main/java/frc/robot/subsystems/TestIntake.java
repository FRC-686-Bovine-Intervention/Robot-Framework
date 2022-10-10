package frc.robot.subsystems;

public class TestIntake extends SubsystemBase {
    private static TestIntake instance;
    public static TestIntake getInstance() {if(instance == null){instance = new TestIntake();}return instance;}

    private TestIntakeCommand intakeCmd;

    private TestIntake()
    {
        loop = TestIntakeLoop.getInstance();
        status = TestIntakeStatus.getInstance();
    }

    public TestIntake setIntakeCommand(TestIntakeCommand intakeCmd) {this.intakeCmd = intakeCmd; return this;}
    public TestIntakeCommand getIntakeCommand()                     {return intakeCmd;}
}
