package frc.robot.auto.actions;

import frc.robot.lib.util.DataLogger;
import frc.robot.subsystems.ExampleSolenoidSubsystem;

public class ExamplePneumaticsRetractAction implements Action
{
	ExampleSolenoidSubsystem subsystem = ExampleSolenoidSubsystem.getInstance();

	public ExamplePneumaticsRetractAction() {}

	@Override
	public void start()
	{
		// extend at start of action
		subsystem.retract();
	}

	@Override
	public boolean isFinished()
	{
		// action is immediately finished
		return true;
	}

	@Override
	public void update()
	{
		// will never be called because action is immediately finished
	}

	@Override
	public void done()
	{
		// nothing to do after finished
	}

	private final DataLogger logger = new DataLogger()
	{
		@Override
		public void log()
		{
		}
	};

	@Override
	public DataLogger getLogger()
	{
		return logger;
	}

}
