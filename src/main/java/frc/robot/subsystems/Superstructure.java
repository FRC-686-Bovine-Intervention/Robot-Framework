package frc.robot.subsystems;

import frc.robot.lib.util.DataLogger;

/**
 * The superstructure subsystem is the overarching superclass containing all components of the superstructure: the
 * intake, hopper, feeder, shooter and LEDs. The superstructure subsystem also contains some miscellaneous hardware that
 * is located in the superstructure but isn't part of any other subsystems like the compressor, pressure sensor, and
 * hopper wall pistons.
 * <p>
 * Instead of interacting with subsystems like the feeder and intake directly, the {@link Robot} class interacts with
 * the superstructure, which passes on the commands to the correct subsystem.
 * <p>
 * The superstructure also coordinates actions between different subsystems like the feeder and shooter.
 *
 * @see Subsystem
 */
public class Superstructure extends Subsystem
{

	// singleton class
	private static Superstructure instance = null;

	public static Superstructure getInstance()
	{
		if (instance == null)
		{
			instance = new Superstructure();
		}
		return instance;
	}

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
	public void zeroSensors()
	{
	}

    @Override
    public boolean checkSystem()
    {
		// TODO: implement checkSystem
        return true;
    }

    @Override
    public void outputTelemetry()
    {
		// TODO: implement outputTelemetry
    }

	private final DataLogger logger = new DataLogger()
	{
		@Override
		public void log()
		{
		}

	};

	public DataLogger getLogger()
	{
		return logger;
	}

}
