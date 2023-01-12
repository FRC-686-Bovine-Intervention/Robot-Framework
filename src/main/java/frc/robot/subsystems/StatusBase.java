package frc.robot.subsystems;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import io.github.oblarg.oblog.Loggable;

public abstract class StatusBase implements LoggableInputs, Loggable{
    public enum EnabledState
    {
        Starting(true),
        Enabled(false),
        Stopping(true),
        Disabled(false);
        public boolean IsInitState;
        private EnabledState(boolean IsInitState)
        {
            this.IsInitState = IsInitState;
        }
    }
    public EnabledState Enabled = EnabledState.Disabled;

    @Override
    public void toLog(LogTable table)
    {
        table.put(this.getClass().getName() + "/Enabled State", Enabled.name());
        exportToTable(table);
    }
    @Override
    public void fromLog(LogTable table)
    {
        switch(table.getString(this.getClass().getName() + "/Enabled State", "Default"))
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
        importFromTable(table);
    }
    public abstract void exportToTable(LogTable table);
    public abstract void importFromTable(LogTable table);
    public abstract void recordOutputs();
}
