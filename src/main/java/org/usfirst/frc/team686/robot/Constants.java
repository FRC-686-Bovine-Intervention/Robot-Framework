package org.usfirst.frc.team686.robot;

import org.usfirst.frc.team686.robot.lib.util.ConstantsBase;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;

/**
 * Attribution: adapted from FRC Team 254
 */


/**
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants determined through calibrations.
 */
public class Constants extends ConstantsBase
{
    private static ConstantsBase mInstance = new Constants();	// make sure we call constructor to set all robot-specific constants
    public static ConstantsBase getInstance() { return mInstance; }
	
    public static double kLoopDt = 0.01;
    public static double kDriveWatchdogTimerThreshold = 0.500;    
    public static int kTalonTimeoutMs = 5;	// ms
    public static int kTalonPidIdx = 0;		// 0 for non-cascaded PIDs, 1 for cascaded PIDs
    	
    public static double kNominalBatteryVoltage = 12.0;
    
    
    // Bumpers
    public static double kCenterToFrontBumper 		= 18.0;	// position of front bumper with respect to robot center of rotation
    public static double kCenterToExtendedIntake 	= 18.0;	// position of intake sweetspot when extended with respect to robot center of rotation
    public static double kCenterToRearBumper 		= 18.0;	// position of rear bumper with respect to robot center of rotation
    public static double kCenterToSideBumper 		= 18.0;	// position of side bumper with respect to robot center of rotation
	public static double kCenterToCornerBumper 		= 18.0;

	// Wheels
    public static double kDriveWheelCircumInches;
    public static double kDriveWheelDiameterInches;
    public static double kTrackLengthInches;
    public static double kTrackWidthInches;
    public static double kTrackEffectiveDiameter;
    public static double kTrackScrubFactor;

    // Wheel Encoder
    public static double kQuadEncoderGain ;	// number of drive shaft rotations per encoder shaft rotation
    
    public static int    kQuadEncoderCodesPerRev;
    public static int    kQuadEncoderUnitsPerRev;
    public static double kQuadEncoderStatusFramePeriod = 0.100;	// 100 ms
    
    // CONTROL LOOP GAINS
    
    public static double kDriveSecondsFromNeutralToFull = 0.375;		// decrease acceleration (reduces current, robot tipping)
    
    // PID gains for drive velocity loop (sent to Talon)
    // Units: error is 4*256 counts/rev. Max output is +/- 1023 units.
    public static double kDriveVelocityKp;
    public static double kDriveVelocityKi;
    public static double kDriveVelocityKd;
    public static double kDriveVelocityKf;
    public static int    kDriveVelocityIZone;
    public static double kDriveVelocityRampRate;
    public static int    kDriveVelocityAllowableError;

    // PID gains for drive base lock loop
    // Units: error is 4*256 counts/rev. Max output is +/- 1023 units.
    public static double kDriveBaseLockKp;
    public static double kDriveBaseLockKi;
    public static double kDriveBaseLockKd;
    public static double kDriveBaseLockKf;
    public static int    kDriveBaseLockIZone;
    public static double kDriveBaseLockRampRate;
    public static int    kDriveBaseLockAllowableError;

    // PID gains for constant heading velocity control
    // Units: Error is degrees. Output is inches/second difference to
    // left/right.
    public static double kDriveHeadingVelocityKp;
    public static double kDriveHeadingVelocityKi;
    public static double kDriveHeadingVelocityKd;
    
    // Point Turn constants
    public static double kPointTurnKp = 0.05;
    public static double kPointTurnKd = 0.50;
    public static double kPointTurnKi = 0.00;
    public static double kPointTurnKf = 0.00;
    public static double kPointTurnCompletionToleranceDeg = 3.0;
    public static double kPointTurnMaxOutput = 0.7; 
    
    // Path following constants
    public static double kPathFollowingMaxVel; // inches/sec  
    public static double kPathFollowingAccelTime = 0.5;	// sec to reach max velocity
    public static double kPathFollowingMaxAccel; // inches/sec^2	
    public static double kPathFollowingLookahead ; // inches
    public static double kPathFollowingCompletionTolerance;
    
    public static double kCollisionVel 			= 24;
    public static double kCollisionAccelTime = 0.5;	// sec to reach max velocity
    public static double kCollisionAccel 		= kCollisionVel / kCollisionAccelTime;
    public static double kCollisionJerkThreshold 	= 0.9;		// maximum JerkY was 0.9 for a 24 inch/sec collision into wall (<0.1 when driving normal)
    public static double kCollisionCurrentThreshold = 20;		// threshold to detect stall current
    
    // Vision constants
    public static double kCameraPoseX ;	// camera location with respect to robot center of rotation, +X axis is in direction of travel
    public static double kCameraPoseY;	// camera location with respect to robot center of rotation, +Y axis is positive to the left
    public static double kCameraPoseTheta;	// camera angle with respect to robot heading
    
    public static double kVisionMaxVel; // inches/sec  		
    public static double kVisionMaxAccel; // inches/sec^2		
    public static double kTargetWidthInches;
    public static double kPegTargetDistanceThresholdFromBumperInches;		// inches to stop from target, measured from front bumper
    public static double kPegTargetDistanceThresholdFromCameraInches;
    public static double kVisionCompletionTolerance; 
    public static double kVisionMaxDistanceInches;		// ignore targets greater than this distance
    public static double kVisionLookaheadDist;	// inches
    public static double kCameraFOVDegrees;			// Camera Field of View (degrees)
    public static double kCameraHalfFOVRadians;			// Half of Camera Field of View (radians)
    public static double kTangentCameraHalfFOV;
    public static double kCameraLatencySeconds;			// Camera image capturing latency
    public static double kTargetLocationFilterConstant;		// 30 time constants in 1 second
    

    // Do not change anything after this line!
    
    // Motor Controllers
    // (Note that if multiple Talons are dedicated to a mechanism, any sensors are attached to the master)
	public static int kRightMotorMasterTalonId;
	public static int kRightMotorSlave1TalonId;
	public static int kRightMotorSlave2TalonId;
    public static int kLeftMotorMasterTalonId;
	public static int kLeftMotorSlave1TalonId;
	public static int kLeftMotorSlave2TalonId;

    // left motors are inverted
    public static boolean	kLeftMotorInverted;
    public static boolean	kRightMotorInverted;
    public static boolean	kLeftMotorSensorPhase;
    public static boolean	kRightMotorSensorPhase;

	public static int kDriveTrainCurrentLimit;
	

    // Joystick Controls
    public static int kXboxButtonA  = 1;
    public static int kXboxButtonB  = 2;
    public static int kXboxButtonX  = 3;
    public static int kXboxButtonY  = 4;
    public static int kXboxButtonLB = 5;
    public static int kXboxButtonRB = 6;
    
    public static int kXboxLStickXAxis  = 0;
    public static int kXboxLStickYAxis  = 1;
    public static int kXboxLTriggerAxis = 2;
    public static int kXboxRTriggerAxis = 3;
    public static int kXboxRStickXAxis  = 4;
    public static int kXboxRStickYAxis  = 5;

	public static int kIntakeButton 			= kXboxButtonRB;
	public static int kOuttakeButton 			= kXboxButtonLB;
	public static int kQuickTurnButton 			= kXboxButtonX;

	
    //Robot stops when joystick axis < 0.1 and >-0.1
    public static double kDriveDeadzone = 0.2;

    // Relay Ports
    public static int kLedRelayPort = 0;
    
    // Gyro
    public enum GyroSelectionEnum { BNO055, NAVX; }
    public static GyroSelectionEnum GyroSelection = GyroSelectionEnum.NAVX;

	// The I2C port the BNO055 is connected to
    public static final I2C.Port BNO055_PORT = I2C.Port.kOnboard;
    
    // BNO055 accelerometer calibration constants
    // ( -7, -34,  33, -24) - taken 10/14/2016
    // (-13, -53,  18, -24) - taken 10/14/2016
    // (  0, -59,  25, -24) - taken 10/14/2016
    // using average of the above
    public static short kAccelOffsetX =  -7;
    public static short kAccelOffsetY = -53;
    public static short kAccelOffsetZ =   25;
    public static short kAccelRadius  = -24;
    
    // The SPI port the NavX is connected to
    // (see https://www.pdocs.kauailabs.com/navx-mxp/guidance/selecting-an-interface/)
    public static final SPI.Port NAVX_PORT = SPI.Port.kMXP;						// the SPI port has low latency (<0.1 ms)

    public static byte NAVX_UPDATE_RATE = (byte) (1.0 / Constants.kLoopDt);		// the SPI port supports update rates from 4-200 Hz
   
    
    
    
    
    
    
    
    			GyroSelection = GyroSelectionEnum.NAVX;

    			kCenterToFrontBumper = 25.0;	// position of front bumper with respect to robot center of rotation
       		    kCenterToRearBumper = 16.0;	// position of rear bumper with respect to robot center of rotation
       		    kCenterToSideBumper = 17.5;	// position of side bumper with respect to robot center of rotation
    		    kCenterToExtendedIntake = 20.0;	//measure distance from the front bumper and add it to this value then change this value
       		    

    			kDriveWheelCircumInches = 18.800 * (244.0/241.72);	// empirically corrected over a 20' test run
    		    kTrackLengthInches = 11.500;	// 23.000 counting the omniwheels
    		    kTrackWidthInches = 21.500;
    		    kTrackScrubFactor = 0.5;

    		    // Wheel Encoder
    		    kQuadEncoderGain = 1.0;			// number of drive shaft rotations per encoder shaft rotation
    											// single speed, double reduction encoder is directly coupled to the drive shaft 
    		    kQuadEncoderCodesPerRev = 64;
     		    
    		    // CONTROL LOOP GAINS
    		    double kNominalEncoderPulsePer100ms = 85;		// velocity at a nominal throttle (measured using NI web interface)
    		    double kNominalPercentOutput 		 = 0.4447;	// percent output of motor at above throttle (using NI web interface)
    		    
    		    
    		    kIntakeLeftMotorInverted = true;
    		    kIntakeRightMotorInverted = false;
    		    
    		    kDriveVelocityKp = 20.0;
    		    kDriveVelocityKi = 0.01;
    		    kDriveVelocityKd = 70.0;
    		    kDriveVelocityKf = kNominalPercentOutput * 1023.0 / kNominalEncoderPulsePer100ms;
    		    kDriveVelocityIZone = 0;
    		    kDriveVelocityRampRate = 0.375;
    		    kDriveVelocityAllowableError = 0;
 
    		    // PID gains for drive base lock loop
    		    // Units: error is 4*256 counts/rev. Max output is +/- 1023 units.
    		    kDriveBaseLockKp = 0.5;
    		    kDriveBaseLockKi = 0;
    		    kDriveBaseLockKd = 0;
    		    kDriveBaseLockKf = 0;
    		    kDriveBaseLockIZone = 0;
    		    kDriveBaseLockRampRate = 0;
    		    kDriveBaseLockAllowableError = 10;

    		    // PID gains for constant heading velocity control
    		    // Units: Error is degrees. Output is inches/second difference to
    		    // left/right.
    		    kDriveHeadingVelocityKp = 4.0;
    		    kDriveHeadingVelocityKi = 0.0;
    		    kDriveHeadingVelocityKd = 50.0;
    		    
    		    
    		    // Motor Controllers
    		    // (Note that if multiple Talons are dedicated to a mechanism, any sensors are attached to the master)

    		    kLeftMotorMasterTalonId     = 1;
    			kLeftMotorSlave1TalonId 	= 2;
    			kElevatorTalonId 			= 3;
    			kRightMotorMasterTalonId 	= 4;
    			kRightMotorSlave1TalonId 	= 5;
    			kArmBarTalonId 				= 6;
    			
    			kLeftIntakePwmChannel		= 0;
    			kRightIntakePwmChannel		= 1;
    			kIntakeSolenoidForwardChannel = 0;
    			kIntakeSolenoidReverseChannel = 1;

    		    // left motors are inverted
    		    kLeftMotorInverted  = false;
    		    kRightMotorInverted = true;
    		    kLeftMotorSensorPhase = false;
    		    kRightMotorSensorPhase = false;
    		    
    			kDriveTrainCurrentLimit = 25;
    			
    			kElevatorLimitSwitchPwmId = 5;
    			kArmBarLimitSwitchPwmId = 1;
    			
    			break;
    			
    			
    			
    			
    			
    			
    			
    			
    		case PRACTICE_BOT:
//    			GyroSelection = GyroSelectionEnum.BNO055;
    			GyroSelection = GyroSelectionEnum.NAVX;
    			
       		    kCenterToFrontBumper = 19.0;	// position of front bumper with respect to robot center of rotation
    		    kCenterToRearBumper = 19.5;	// position of rear bumper with respect to robot center of rotation
    		    kCenterToSideBumper = 17.5;	// position of side bumper with respect to robot center of rotation
    		    kCenterToExtendedIntake = 24.0;
    		    
     		    kDriveWheelCircumInches = 13.229;//13.250;
    		    kTrackLengthInches = 25.000;
    		    kTrackWidthInches = 23.000;
    		    kTrackScrubFactor = 0.5;

    		    // Wheel Encoder
    		    kQuadEncoderGain = ( 30.0 / 54.0 ) * ( 12.0 / 36.0 );	// number of drive shaft rotations per encoder shaft rotation
    																	// 54:30 drive shaft --> 3rd stage, 36:12 3rd stage --> encoder shaft     
    		    kQuadEncoderCodesPerRev = 64;
    		    
    		    // CONTROL LOOP GAINS
    		    kNominalEncoderPulsePer100ms = 900; //290;		// velocity at a nominal throttle (measured using NI web interface)
    		    kNominalPercentOutput 		 = 0.4995;	// percent output of motor at above throttle (using NI web interface)
    		    
    		    
    		    
    		    kIntakeLeftMotorInverted = false;
    		    kIntakeRightMotorInverted = true;

System.out.printf("PRACTICE_BOT: kIntakeLeftMotorInverted = %b (should be false)\n", kIntakeLeftMotorInverted);    		    
    		    
    		    kDriveVelocityKp = 2.0;
    		    kDriveVelocityKi = 0.001;
    		    kDriveVelocityKd = 100.0;
    		    kDriveVelocityKf = kNominalPercentOutput * 1023.0 / kNominalEncoderPulsePer100ms;
    		    kDriveVelocityIZone = 0;
    		    kDriveVelocityRampRate = 0.0;
    		    kDriveVelocityAllowableError = 0;

    		    // PID gains for drive base lock loop
    		    // Units: error is 4*256 counts/rev. Max output is +/- 1023 units.
    		    kDriveBaseLockKp = 0.5;
    		    kDriveBaseLockKi = 0;
    		    kDriveBaseLockKd = 0;
    		    kDriveBaseLockKf = 0;
    		    kDriveBaseLockIZone = 0;
    		    kDriveBaseLockRampRate = 0;
    		    kDriveBaseLockAllowableError = 10;

    		    // PID gains for constant heading velocity control
    		    // Units: Error is degrees. Output is inches/second difference to
    		    // left/right.
    		    kDriveHeadingVelocityKp = 0.4;
    		    kDriveHeadingVelocityKi = 0.0;
    		    kDriveHeadingVelocityKd = 5.0;
    		    
    		    
    		    // Motor Controllers
    		    // (Note that if multiple Talons are dedicated to a mechanism, any sensors are attached to the master)
  
    		    
    		    kLeftMotorMasterTalonId 	= 1;
    			kLeftMotorSlave1TalonId 	= 2;
    			kElevatorTalonId 			= 4;
    			kRightMotorMasterTalonId 	= 5;
    			kRightMotorSlave1TalonId 	= 3;
    			kArmBarTalonId 				= 6;
    			
    			
    			kLeftIntakePwmChannel		= 0;
    			kRightIntakePwmChannel		= 1;
    			kIntakeSolenoidForwardChannel = 0;
    			kIntakeSolenoidReverseChannel = 1;

    		    // left motors are inverted
    		    kLeftMotorInverted  = true;
    		    kRightMotorInverted = false;
    		    kLeftMotorSensorPhase = true;
    		    kRightMotorSensorPhase = true;
    			
    			kDriveTrainCurrentLimit = 25;
    			
    			kElevatorLimitSwitchPwmId = 0;
    			kArmBarLimitSwitchPwmId = 1;
    			
    		    break;
    	}

    	// calculated constants
    	kCenterToCornerBumper = Math.sqrt(kCenterToRearBumper*kCenterToRearBumper + kCenterToSideBumper*kCenterToSideBumper);
    	
	    kDriveWheelDiameterInches = kDriveWheelCircumInches / Math.PI;
	    kTrackEffectiveDiameter = (kTrackWidthInches * kTrackWidthInches + kTrackLengthInches * kTrackLengthInches) / kTrackWidthInches;

	    kQuadEncoderUnitsPerRev = (int)(4*kQuadEncoderCodesPerRev / kQuadEncoderGain);    
	    
	    // Path following constants
	    kPathFollowingMaxVel    = 72.0; // inches/sec  		
	    kPathFollowingAccelTime = 0.5;
	    kPathFollowingMaxAccel  = kPathFollowingMaxVel / kPathFollowingAccelTime; // inches/sec^2	 
	    kPathFollowingLookahead = 24.0; // inches
	    kPathFollowingCompletionTolerance = 4.0; 
	    
	    // Vision constants
	    kCameraPoseX     = +7.25;	// camera location with respect to robot center of rotation, +X axis is in direction of travel
	    kCameraPoseY     =     0;	// camera location with respect to robot center of rotation, +Y axis is positive to the left
	    kCameraPoseTheta =     0;	// camera angle with respect to robot heading
	    
	    kVisionMaxVel    = 20.0; // inches/sec  		
	    kVisionMaxAccel  = 20.0; // inches/sec^2		
	    kTargetWidthInches = 10.25;
	    kPegTargetDistanceThresholdFromBumperInches = 18;		// inches to stop from target, measured from front bumper
	    kPegTargetDistanceThresholdFromCameraInches = kCenterToFrontBumper - kCameraPoseX + kPegTargetDistanceThresholdFromBumperInches;
	    kVisionCompletionTolerance = 1.0; 
	    kVisionMaxDistanceInches = 240;		// ignore targets greater than this distance
	    kVisionLookaheadDist = 24.0;	// inches
	    kCameraFOVDegrees = 42.5;			// Camera Field of View (degrees)
	    kCameraHalfFOVRadians = kCameraFOVDegrees/2.0 * Math.PI/180.0;			// Half of Camera Field of View (radians)
	    kTangentCameraHalfFOV = Math.tan(kCameraHalfFOVRadians);
	    kCameraLatencySeconds = 0.240;			// Camera image capturing latency
	    kTargetLocationFilterConstant = (30.0 * kLoopDt);		// 30 time constants in 1 second
    
	    if (cubeInProximitySensor == null)
	    {
	    	
	    	cubeInProximitySensor = new DigitalInput(Constants.kCubeInProximitySensorPort);
	    }
	   
//	    if (cubeCloseProximitySensor == null)
//	    {
//	    	cubeCloseProximitySensor = new DigitalInput(Constants.kCubeCloseProximitySensorPort);
//	    }
	    
    }
}
