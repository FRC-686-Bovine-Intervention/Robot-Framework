package org.usfirst.frc.team686.robot.subsystems;

import org.usfirst.frc.team686.robot.lib.util.DataLogger;

public class Superstructure extends Subsystem {
	
	private static Superstructure instance = new Superstructure();
	public static Superstructure getInstance() { return instance; }
	
	private Superstructure()
	{
	}
	
	public void disable()
	{
	}
	
	public void enable()
	{
	}

	
	  
	@Override
	public void stop()
	{
	}

	@Override
	public void zeroSensors() {}

	
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
        }
        
    };
    
    public DataLogger getLogger() { return logger; }
	
	
}
