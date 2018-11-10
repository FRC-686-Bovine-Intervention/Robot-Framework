package org.usfirst.frc.team686.robot.auto.modes;

import org.usfirst.frc.team686.robot.Constants;
import org.usfirst.frc.team686.robot.auto.AutoModeBase;
import org.usfirst.frc.team686.robot.auto.AutoModeEndedException;
import org.usfirst.frc.team686.robot.auto.actions.PathFollowerAction;
import org.usfirst.frc.team686.robot.lib.util.Path;
import org.usfirst.frc.team686.robot.lib.util.PathSegment;
import org.usfirst.frc.team686.robot.lib.util.Vector2d;
import org.usfirst.frc.team686.robot.lib.util.Path.Waypoint;

/**
 * Go over the defenses in the starting configuration, then launch one ball (in
 * the robot at start)
 */
public class SquarePatternMode extends AutoModeBase {

	Path path;
    public SquarePatternMode(int lane, boolean shouldDriveBack) 
    {
 
    }
    
    private void init(){
    	
    	PathSegment.Options options = new PathSegment.Options(Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel, Constants.kPathFollowingLookahead, false);
    	
    	path = new Path();
    	path.add(new Waypoint(new Vector2d( 0, 0), options));
        path.add(new Waypoint(new Vector2d( 240.0, 0), options));
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
    	System.out.println("Starting Auto Mode: Square Pattern");

    	init();
    	
        //Path path = new Path();
        //path.add(new Waypoint(new Vector2d( 0, 0), options));
        //path.add(new Waypoint(new Vector2d( 240.0, 0), options));
        //path.add(new Waypoint(new Vector2d( 96.0, 72.0), options));
        //path.add(new Waypoint(new Vector2d( 0, 72.0), options));
        //path.add(new Waypoint(new Vector2d( 0, 0), options));
  
        //Path revPath = new Path(path);
        //revPath.setReverseOrder();
        //revPath.setReverseDirection();
        
        runAction(new PathFollowerAction(path));			// drive forward
        //runAction(new PathFollowerWithVisionAction(revPath));    	// drive reversed 
    }
}
