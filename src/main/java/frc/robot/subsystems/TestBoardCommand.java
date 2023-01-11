package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.TestBoardLoop.TestBoardState;

public class TestBoardCommand {
    private TestBoardState state;
    private double commandTime;

    public TestBoardCommand(TestBoardState state)
    {
        this.state = state;
        this.commandTime = Timer.getFPGATimestamp();
    }

    public TestBoardCommand setState(TestBoardState state)
    {
        this.state = state;
        return this;
    }

    public TestBoardState getState()
    {
        return state;
    }

    public double getCommandTime()
    {
        return commandTime;
    }
}
