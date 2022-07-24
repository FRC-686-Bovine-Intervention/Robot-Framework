package frc.robot;

public class Constants {
    private static Constants instance = null;
    public static Constants getInstance() {if(instance == null){instance = new Constants();}return instance;}

    // Hardware Port Definitions
    // Drivetrain Hardware
    public static int kPigeonID =           1;
    public static int kLeftMasterID =       2;
    public static int kLeftSlaveID =        3;
    public static int kRightMasterID =      4;
    public static int kRightSlaveID =       5;
    // Intake Hardware
    public static int kArmMotorID =         6;
    public static int kRollerMotorID =      7;
    // Climber Hardware
    public static int kLeftClimberID =          8;
    public static int kRightClimberID =         9;
    public static int kClimberHallEffectPort =  9;
    // Control Hardware
    public static int kThrustmasterPort =   0;
    public static int kButtonboardPort =    1;

    public static double kLoopDt = 0.01;
    public static int kTalonTimeoutMs = 5;


    // Robot Dimensions
    public static double kCenterToSideBumper = 15.0;
    public static double kCenterToFrontBumper = 19.5;
    public static double kCenterToIntake = 32.0;
}

