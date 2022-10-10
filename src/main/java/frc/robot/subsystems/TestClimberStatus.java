package frc.robot.subsystems;

import java.util.ArrayList;

public class TestClimberStatus extends StatusBase {
    private static TestClimberStatus instance = null;
    public static TestClimberStatus getInstance() {if(instance == null){instance = new TestClimberStatus();}return instance;}

    public enum ClimberState {
        DEFENSE(ClimberPos.RETRACTED),
        LOW_BAR(ClimberPos.RETRACTED),
        EXTEND_GROUND(ClimberPos.EXTENDED),
        SLOW_DRIVE(ClimberPos.EXTENDED),
        RETRACT_EXTEND(ClimberPos.EXTENDED),
        INTAKE(ClimberPos.EXTENDED),
        CALIBRATING(ClimberPos.CALIBRATION),
        CUSTOM(null);
 
        public final ClimberPos pos; 
        ClimberState(ClimberPos pos) {this.pos = pos;}
        public ClimberState nextState()
        {
            switch(this)
            {
                default:                return this;
                case DEFENSE:           return LOW_BAR;
                case LOW_BAR:           return EXTEND_GROUND;
                case EXTEND_GROUND:     return SLOW_DRIVE;
                case SLOW_DRIVE:        return RETRACT_EXTEND;
                case RETRACT_EXTEND:    return INTAKE;
                case INTAKE:            return RETRACT_EXTEND;
            }
        }
    }
    public enum ClimberPos
    {
        EXTENDED(26.75),
        RETRACTED(10),
        CALIBRATION(0);

        public final double distIn;
        ClimberPos(double distIn) {this.distIn = distIn;}
    }

    private TestIntakeCommand intakeCmd;

    private ClimberState status;
    private ArrayList<TestClimberStatus.ClimberState> statusHistory;
    private boolean calibrated;
    private boolean calibrationPaused;

    private double statorCurrent;
    private double outputVoltage;
    private double busVoltage;
    private double supplyCurrent;
    private double currentPos;
    private double intakeInput;

    public synchronized TestIntakeCommand getIntakeCommand() {return intakeCmd;}
    public synchronized TestClimberStatus setIntakeCommand(TestIntakeCommand intakeCmd) {this.intakeCmd = intakeCmd; return this;}
    
    public synchronized ClimberState getStatus() {return status;}
    public synchronized TestClimberStatus setStatus(ClimberState status) {this.status = status; return this;}
    
    public synchronized ArrayList<TestClimberStatus.ClimberState> getStatusHistory() {return statusHistory;}
    public synchronized TestClimberStatus setStatusHistory(ArrayList<TestClimberStatus.ClimberState> statusHistory) {this.statusHistory = statusHistory; return this;}
    
    public synchronized boolean getCalibrated() {return calibrated;}
    public synchronized TestClimberStatus setCalibrated(boolean calibrated) {this.calibrated = calibrated; return this;}
    
    public synchronized boolean getCalibrationPaused() {return calibrationPaused;}
    public synchronized TestClimberStatus setCalibrationPaused(boolean calibrationPaused) {this.calibrationPaused = calibrationPaused; return this;}
    
    public synchronized double getStatorCurrent() {return statorCurrent;}
    public synchronized TestClimberStatus setStatorCurrent(double statorCurrent) {this.statorCurrent = statorCurrent; return this;}
    
    public synchronized double getOutputVoltage() {return outputVoltage;}
    public synchronized TestClimberStatus setOutputVoltage(double outputVoltage) {this.outputVoltage = outputVoltage; return this;}
    
    public synchronized double getBusVoltage() {return busVoltage;}
    public synchronized TestClimberStatus setBusVoltage(double busVoltage) {this.busVoltage = busVoltage; return this;}
    
    public synchronized double getSupplyCurrent() {return supplyCurrent;}
    public synchronized TestClimberStatus setSupplyCurrent(double supplyCurrent) {this.supplyCurrent = supplyCurrent; return this;}
    
    public synchronized double getCurrentPos() {return currentPos;}
    public synchronized TestClimberStatus setCurrentPos(double currentPos) {this.currentPos = currentPos; return this;}
    
    public synchronized double getIntakeInput() {return intakeInput;}
    public synchronized TestClimberStatus setIntakeInput(double intakeInput) {this.intakeInput = intakeInput; return this;}
}
