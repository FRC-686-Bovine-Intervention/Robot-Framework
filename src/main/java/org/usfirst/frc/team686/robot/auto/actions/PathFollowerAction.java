package org.usfirst.frc.team686.robot.auto.actions;

import org.usfirst.frc.team686.robot.command_status.RobotState;
import org.usfirst.frc.team686.robot.lib.util.DataLogger;
import org.usfirst.frc.team686.robot.lib.util.Path;
import org.usfirst.frc.team686.robot.lib.util.PathFollower;
import org.usfirst.frc.team686.robot.lib.util.PathFollower.PathVisionState;

/**
 * Action for following a path defined by a Path object.
 * 
 * Serially configures a PathFollower object to follow each path 
 */
public class PathFollowerAction implements Action  
{
	PathFollower driveCtrl;
	Path path;

    public PathFollowerAction(Path _path) 
    {
    	driveCtrl = new PathFollower(_path, PathVisionState.PATH_FOLLOWING);
    	
    	path = _path;
    }

    public PathFollower getDriveController() { return driveCtrl; }

    @Override
    public void start() 
    {
		System.out.println("PathFollowerAction.start(), pose = " + RobotState.getInstance().getLatestFieldToVehicle().toString());
		driveCtrl.start();
    }


    @Override
    public void update() 
    {
    	driveCtrl.update();
	}	
	
	
    @Override
    public boolean isFinished() 
    {
    	boolean finished = driveCtrl.isFinished();
    	
//		if (finished)
//			System.out.println("InterruptableAction Finished");
	
    	return finished;
    }

    @Override
    public void done() 
    {
		System.out.println("PathFollowerAction.done(),  pose = " + RobotState.getInstance().getLatestFieldToVehicle().toString());
		// cleanup code, if any
		driveCtrl.done();
    }

 
    
    
    public DataLogger getLogger() { return driveCtrl.getLogger(); }
}
