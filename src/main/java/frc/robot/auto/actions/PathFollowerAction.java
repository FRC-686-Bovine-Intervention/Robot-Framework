package frc.robot.auto.actions;

import frc.robot.lib.util.Path;
import frc.robot.lib.util.PathFollower;
import frc.robot.lib.util.PathFollower.PathVisionState;

public class PathFollowerAction implements Action{
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
      driveCtrl.start();
    }


    @Override
    public void run() 
    {
    	driveCtrl.update();
	  }	
	
	
    @Override
    public boolean isFinished() 
    {
    	boolean finished = driveCtrl.isFinished();
      return finished;    	
    }

    @Override
    public void done() 
    {
      // cleanup code, if any
      driveCtrl.done();
    }
}
