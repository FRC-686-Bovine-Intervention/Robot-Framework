package frc.robot.auto.modes;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.lib.util.Pose;
import frc.robot.lib.util.Vector2d;

/**
 * Interface that holds all the field measurements 
 */

public class FieldDimensions 
{
	// dimensions of field components
	public static final double kFieldLengthX = 648;       // 54'
	public static final double kFieldLengthY = 324;       // 27'
    
    // Fender (left/right as viewed from the driver station)
    public static final double originToFenderCenterInches = 34.0;
    public static final Vector2d fenderCenter = new Vector2d(0,0);  // all autonomous coordinates are relative to center of fender
    public static final double fenderShotAngleRad = Units.degreesToRadians(180.0);  // shoot facing towards center of hub

    // Fender Shot Positions
    public static final Vector2d fenderShotPos     = new Vector2d(Constants.kCenterToFrontBumper, 0);
    public static final Vector2d fenderApproachPos = new Vector2d(48.0, 0);     // some distance in front of fender
    public static final Vector2d fenderBackupPos   = new Vector2d(40.0, 0);     // some distance in front of fender

    //====================================================================================================
    // 2 Ball Auto 
    //====================================================================================================
    
    // Starting Pose
    public static final Vector2d tarmacApex = new Vector2d(82.75, 0);   // outside of tape at (84.75, 0)
    public static final double tarmacTapeAngleRad = Units.degreesToRadians(67.5); 
    public static final double twoBallAutoInitialHeadingRad = Units.degreesToRadians(-22.5);
    public static final Pose twoBallAutoStartingPose = new Pose(new Vector2d(tarmacApex
                                    .sub(Vector2d.magnitudeAngle(Constants.kCenterToSideBumper, tarmacTapeAngleRad))
                                    .sub(Vector2d.magnitudeAngle(Constants.kCenterToFrontBumper, twoBallAutoInitialHeadingRad))), 
                                    twoBallAutoInitialHeadingRad);
    
    // Ball Positions
    public static final Vector2d redBall2of2  = new Vector2d(115.5, -31.5);   // near hangar
    public static final Vector2d blueBall2of2  = new Vector2d(116, -29);   // near hangar
    public static Vector2d[] ball2of2  = new Vector2d[]{redBall2of2, blueBall2of2};
    public static final Vector2d redTheirBall = new Vector2d(91, -85);    // left wall
    public static final Vector2d blueTheirBall = new Vector2d(93.5, -84);    // left wall
    public static Vector2d[] theirBall  = new Vector2d[]{redTheirBall, blueTheirBall};//DriverStation.getAlliance() == Alliance.Red ? redBall2of2 : blueBall2of2;   // near hangar
    public static final Vector2d twoBallAutoFinalTarget  = new Vector2d(51, -127);   // other side of field, left wall
    
    // Hangar Shot Position
    public static final Vector2d hangarShotPos = new Vector2d(186, -36);
    
    //====================================================================================================
    // 3 Ball Auto 
    //====================================================================================================
    
    // Starting Pose
    public static final Pose threeBallAutoStartingPose = new Pose(fenderShotPos, Units.degreesToRadians(180));
    
    // Ball Positions
    public static final Vector2d redBall2of3   = new Vector2d(116, 31);   // right wall
    public static final Vector2d blueBall2of3   = new Vector2d(116, 28.5);   // right wall
    public static final Vector2d[] ball2of3 = new Vector2d[]{redBall2of3,blueBall2of3};
    public static final Vector2d redBall3of3   = new Vector2d(97, -83);   // near center
    public static final Vector2d blueBall3of3   = new Vector2d(92, -86);   // near center
    public static final Vector2d[] ball3of3 = new Vector2d[]{redBall3of3,blueBall3of3};
    public static final Vector2d threeBallAutoFinalTarget  = new Vector2d(51, 127);   // other side of field, left wall
    
}