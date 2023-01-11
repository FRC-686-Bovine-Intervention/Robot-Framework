package frc.robot.subsystems;

import frc.robot.subsystems.TestBoardLoop.TestBoardState;

public class TestBoard extends SubsystemBase {
    private static TestBoard instance;
    public static TestBoard getInstance() {if(instance == null){instance = new TestBoard();} return instance;}
    @Override
    public void init() {
        loop = TestBoardLoop.getInstance();
        status = TestBoardStatus.getInstance();
        HAL = TestBoardHAL.getInstance();
    }

    private TestBoardCommand command;

    public TestBoard setCommand(TestBoardCommand command) {this.command = command; return this;}
    public TestBoardCommand getCommand() {
        if(command == null)
            command = new TestBoardCommand(TestBoardState.NEUTRAL);
        return command;
    }
}
