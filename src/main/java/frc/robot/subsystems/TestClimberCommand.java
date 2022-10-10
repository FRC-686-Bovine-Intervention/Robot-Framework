package frc.robot.subsystems;

import frc.robot.subsystems.TestClimberStatus.ClimberState;

public class TestClimberCommand {
    private ClimberState state = null;
    private double power = 0;
    private int stateChange = 0;

    private TestClimberCommand(){}

    public static TestClimberCommand fromState(ClimberState state)
    {
        return new TestClimberCommand().setState(state);
    }

    public static TestClimberCommand fromButtons(boolean nextButton, boolean prevButton) {return fromButtons(nextButton, prevButton, false);}
    public static TestClimberCommand fromButtons(boolean nextButton, boolean prevButton, boolean resetButton)
    {
        int netChange = 0;
        if(nextButton)  {netChange += 1;}
        if(prevButton)  {netChange -= 1;}
        if(resetButton) {netChange = Integer.MIN_VALUE;}
        return new TestClimberCommand().setStateChange(netChange);
    }

    public TestClimberCommand setState(ClimberState state)      {this.state = state; return this;}
    public ClimberState getState()                              {return state;}

    public TestClimberCommand setStateChange(int stateChange)   {this.stateChange = stateChange; return this;}
    public int getStateChange()                                 {return stateChange;}

    public TestClimberCommand setPower(double power)            {this.power = power; return this;}
    public double getPower()                                    {return power;}
}
