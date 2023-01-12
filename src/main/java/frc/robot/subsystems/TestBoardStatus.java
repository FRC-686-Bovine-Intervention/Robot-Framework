package frc.robot.subsystems;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.TestBoardLoop.TestBoardState;
import io.github.oblarg.oblog.annotations.Log;

public class TestBoardStatus extends StatusBase {
    private static TestBoardStatus instance;
    public static TestBoardStatus getInstance() {if(instance == null){instance = new TestBoardStatus();} return instance;}

    private TestBoardState state;

    @Log
    private double motorStatorCurrent;
    @Log
    private double motorOutputVoltage;

    public double getStatorCurrent() {return motorStatorCurrent;}
    public TestBoardStatus setStatorCurrent(double statorCurrent)
    {
        this.motorStatorCurrent = statorCurrent;
        return this;
    }
    
    public double getOutputVoltage() {return motorOutputVoltage;}
    public TestBoardStatus setOutputVoltage(double outputVoltage)
    {
        this.motorOutputVoltage = outputVoltage;
        return this;
    }

    public TestBoardState getState() {return state;}
    public TestBoardStatus setState(TestBoardState state)
    {
        this.state = state;
        return this;
    }

    @Override
    public void exportToTable(LogTable table) {
        table.put("TestBoard/Inputs/Motor Stator Current", motorStatorCurrent);
        table.put("TestBoard/Inputs/Motor Output Voltage", motorOutputVoltage);
    }
    @Override
    public void importFromTable(LogTable table) {
        motorStatorCurrent = table.getDouble("TestBoard/Inputs/Motor Stator Current", -686);
        motorOutputVoltage = table.getDouble("TestBoard/Inputs/Motor Output Voltage", -686);
    }
    @Override
    public void recordOutputs() {
        Logger.getInstance().recordOutput("TestBoard/Outputs/State", state.name());
        Logger.getInstance().recordOutput("TestBoard/Outputs/Motor Setpoint", state.motorPower);
    }
}
