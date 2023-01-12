package frc.robot.subsystems;

import io.github.oblarg.oblog.Loggable;

public abstract class SubsystemBase implements Loggable{
    public LoopBase loop;
    public StatusBase status;
    public HALBase HAL;

    /**
     * DO NOT SET SUPER VARIABLES IN A CONSTRUCTOR<p>
     * Instead do it here<p>
     * Setting super variables in a constructor will cause a recursive .getInstance() loop
     */
    public abstract void init();
}
