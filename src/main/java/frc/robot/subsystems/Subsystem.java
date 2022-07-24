package frc.robot.subsystems;

import frc.robot.loops.Loop;

public abstract class Subsystem {
    public Loop controlLoop;
    public Loop statusLoop;
    public Status status;
}
