package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;

public class TestBoardLoop extends LoopBase {
    private static TestBoardLoop instance;
    public static TestBoardLoop getInstance() {if(instance == null){instance = new TestBoardLoop();} return instance;}

    private final TestBoard testBoard;
    private final TestBoardStatus status;
    private final TestBoardHAL HAL;

    // private final GenericEntry stateEntry;
    // private final GenericEntry motorVoltageEntry;
    // private final GenericEntry motorCurrentEntry;

    private static final double kCommandTimeoutThreshold = 1;

    public enum TestBoardState
    {
        NEUTRAL(0),
        FORWARD(0.25),
        BACKWARD(-0.25),
        FAST_FORWARD(0.5);

        public final double motorPower;
        TestBoardState(double motorPower)
        {
            this.motorPower = motorPower;
        }
    }

    private TestBoardState state = TestBoardState.NEUTRAL;

    private TestBoardLoop()
    {
        testBoard = TestBoard.getInstance();
        HAL = TestBoardHAL.getInstance();
        status = TestBoardStatus.getInstance();

        // tab = Shuffleboard.getTab("TestBoard");
        // stateEntry = tab.add("State", "not updating").getEntry();
        // motorVoltageEntry = tab.add("Voltage", -686).getEntry();
        // motorCurrentEntry = tab.add("Current", -686).getEntry();

        _Subsystem = testBoard;
    }

    @Override
    public void Enabled() {
        TestBoardCommand newCmd = testBoard.getCommand();

        if(Timer.getFPGATimestamp() > newCmd.getCommandTime() + kCommandTimeoutThreshold)
        {
            state = TestBoardState.NEUTRAL;
        }
        else
        {
            state = newCmd.getState();
        }

        HAL.setMotorPower(state.motorPower);
    }

    @Override
    public void Disabled() {
        
    }

    @Override
    public void Update() {
        status.setState(state);

        // stateEntry.setString(status.getState().name());
        // motorVoltageEntry.setDouble(status.getOutputVoltage());
        // motorCurrentEntry.setDouble(status.getStatorCurrent());
    }

    @Override
    public void UpdateStatus() {
        status.setOutputVoltage(HAL.getMotorOutputVoltage());
        status.setStatorCurrent(HAL.getStatorCurrent());
    }
}
