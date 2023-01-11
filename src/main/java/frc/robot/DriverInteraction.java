package frc.robot;

import frc.robot.controls.Controls;
import frc.robot.controls.Controls.ButtonControlEnum;
import frc.robot.subsystems.TestBoard;
import frc.robot.subsystems.TestBoardCommand;
import frc.robot.subsystems.TestBoardLoop.TestBoardState;

public class DriverInteraction {
    private static DriverInteraction instance;
    public static DriverInteraction getInstance() {if(instance == null){instance = new DriverInteraction();}return instance;}

    private final Controls controls;
    private final TestBoard testBoard;

    private DriverInteraction()
    {
        controls = Controls.getInstance();
        testBoard = TestBoard.getInstance();
    }
    
    public void run()
    {
        TestBoardCommand cmd = new TestBoardCommand(TestBoardState.NEUTRAL);
        if(controls.getButton(ButtonControlEnum.FORWARD))
            cmd.setState(TestBoardState.FORWARD);
        else if(controls.getButton(ButtonControlEnum.BACKWARD))
            cmd.setState(TestBoardState.BACKWARD);
        else if(controls.getButton(ButtonControlEnum.FAST_FORWARD))
            cmd.setState(TestBoardState.FAST_FORWARD);
        testBoard.setCommand(cmd);
    }
}
