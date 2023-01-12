package frc.robot.subsystems;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import io.github.oblarg.oblog.Loggable;

public abstract class StatusBase implements LoggableInputs, Loggable{

    public SubsystemBase Subsystem;

    public enum EnabledState
    {
        Starting(true),
        Enabled(false),
        Stopping(true),
        Disabled(false);
        public final boolean IsInitState;
        private EnabledState(boolean IsInitState)
        {
            this.IsInitState = IsInitState;
        }
    }
    public EnabledState Enabled = EnabledState.Disabled;

    protected GenericEntry EnabledEntry;
    private boolean EnabledSwitch = true;

    @Override
    public void toLog(LogTable table)
    {
        String prefix = Subsystem.getClass().getSimpleName();
        table.put(prefix + "/Enabled Switch", EnabledSwitch);
        table.put(prefix + "/Enabled State", Enabled.name());
        exportToTable(table, prefix);
    }
    public abstract void exportToTable(LogTable table, String prefix);

    @Override
    public void fromLog(LogTable table)
    {
        String prefix = Subsystem.getClass().getSimpleName();
        EnabledSwitch = table.getBoolean(prefix + "/Enabled Switch", true);
        switch(table.getString(prefix + "/Enabled State", "Default"))
        {
            case "Starting":
                Enabled = EnabledState.Starting;
            break;
            case "Enabled":
                Enabled = EnabledState.Enabled;
            break;
            case "Stopping":
                Enabled = EnabledState.Stopping;
            break;
            case "Disabled":
                Enabled = EnabledState.Disabled;
            break;
            default: break;
        }
        importFromTable(table, prefix);
    }
    public abstract void importFromTable(LogTable table, String prefix);

    public void inputs()
    {
        if(EnabledEntry == null)
            EnabledSwitch = true;
        else
            EnabledSwitch = EnabledEntry.getBoolean(true);
        switch(Enabled)
        {
            case Starting:
                Enabled = EnabledState.Enabled;
            break;
            case Enabled:
                if(DriverStation.isDisabled() || !EnabledSwitch)
                    Enabled = EnabledState.Stopping;
            break;
            case Stopping:
                Enabled = EnabledState.Disabled;
            break;
            case Disabled:
                if(DriverStation.isEnabled() && EnabledSwitch)
                    Enabled = EnabledState.Starting;
            break;
        }
        updateInputs();
    }
    public abstract void updateInputs();

    public void record()
    {
        String prefix = Subsystem.getClass().getSimpleName();
        Logger.getInstance().recordOutput(prefix + "/Enabled Switch", EnabledSwitch);
        Logger.getInstance().recordOutput(prefix + "/Enabled State", Enabled.name());
        recordOutputs(prefix);
    }
    public abstract void recordOutputs(String prefix);
}
