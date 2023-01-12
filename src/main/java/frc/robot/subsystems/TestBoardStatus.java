package frc.robot.subsystems;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.TestBoardLoop.TestBoardState;
import io.github.oblarg.oblog.annotations.Log;

public class TestBoardStatus extends StatusBase {
    private static TestBoardStatus instance;
    public static TestBoardStatus getInstance() {if(instance == null){instance = new TestBoardStatus();} return instance;}

    private TestBoardStatus()
    {
        Subsystem = TestBoard.getInstance();
        EnabledEntry = Shuffleboard.getTab("Test Board").add("Enabled", true).getEntry();
    }

    private final TestBoardHAL HAL = TestBoardHAL.getInstance();

    @Log(name = "Stator Current",tabName = "Test Board",columnIndex = 9,rowIndex = 0)
    private double motorStatorCurrent;
    public double getStatorCurrent() {return motorStatorCurrent;}
    public TestBoardStatus setStatorCurrent(double statorCurrent)
    {
        this.motorStatorCurrent = statorCurrent;
        return this;
    }

    @Log(name = "Output Voltage",tabName = "Test Board",columnIndex = 0,rowIndex = 3)
    private double motorOutputVoltage;
    public double getOutputVoltage() {return motorOutputVoltage;}
    public TestBoardStatus setOutputVoltage(double outputVoltage)
    {
        this.motorOutputVoltage = outputVoltage;
        return this;
    }

    private TestBoardState state;
    public TestBoardState getState() {return state;}
    public TestBoardStatus setState(TestBoardState state)
    {
        this.state = state;
        return this;
    }

    @Override
    public void exportToTable(LogTable table, String prefix) {
        table.put(prefix + "/Motor Stator Current", motorStatorCurrent);
        table.put(prefix + "/Motor Output Voltage", motorOutputVoltage);
    }
    @Override
    public void importFromTable(LogTable table, String prefix) {
        motorStatorCurrent = table.getDouble(prefix + "/Motor Stator Current", -686);
        motorOutputVoltage = table.getDouble(prefix + "/Motor Output Voltage", -686);
    }
    @Override
    public void recordOutputs(String prefix) {
        Logger.getInstance().recordOutput(prefix + "/State", state.name());
        Logger.getInstance().recordOutput(prefix + "/Motor Setpoint", state.motorPower);
    }
    @Override
    public void updateInputs() {
        setOutputVoltage(HAL.getMotorOutputVoltage());
        setStatorCurrent(HAL.getStatorCurrent());
    }
}
