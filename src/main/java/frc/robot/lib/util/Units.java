package frc.robot.lib.util;

import frc.robot.Constants;

public class Units
{
    public static double rpm_to_rads_per_sec(double rpm)
    {
        return rpm * (2.0 * Math.PI) / 60.0;
    }

    public static double rads_per_sec_to_rpm(double rads_per_sec)
    {
        return rads_per_sec * 60.0 / (2.0 * Math.PI);
    }

    public static double inches_to_meters(double inches)
    {
        return inches * 0.0254;
    }

    public static double meters_to_inches(double meters)
    {
        return meters / 0.0254;
    }

    public static double feet_to_meters(double feet)
    {
        return inches_to_meters(feet * 12.0);
    }

    public static double meters_to_feet(double meters)
    {
        return meters_to_inches(meters) / 12.0;
    }

    public static double degrees_to_radians(double degrees)
    {
        return Math.toRadians(degrees);
    }

    public static double radians_to_degrees(double radians)
    {
        return Math.toDegrees(radians);
    }

    public static int radians_to_encoder_units(double radians, double encoderUnitsPerRev){
        return (int)(radians*(encoderUnitsPerRev/(Math.PI*2.0)));
    }

    public static int rads_per_sec_to_units_per_frame(double rps, double encoderUnitsPerRev){
        return (int)(rps*(encoderUnitsPerRev/(Math.PI*2.0))*(Constants.kTalonFramePeriod));
    }

    public static int rads_per_sec2_to_units_per_frame2(double rpsps, double encoderUnitsPerRev){
        return (int)(rpsps*(encoderUnitsPerRev/(Math.PI*2.0))*(Constants.kTalonFramePeriod*Constants.kTalonFramePeriod));
    }

    public static double encoderUnitsToRadians(int encoderUnits, double encoderUnitsPerRev){
        return ((double)encoderUnits*((Math.PI*2.0)/encoderUnitsPerRev));
    }

    public static double units_per_frame_to_rads_per_sec(int upf, double encoderUnitsPerRev){
        return ((double)upf*((Math.PI*2.0)/encoderUnitsPerRev)*(1.0/Constants.kTalonFramePeriod));
    }

    public static double units_per_frame2_to_rads_per_sec2(int upfpf, double encoderUnitsPerRev){
        return ((double)upfpf*((Math.PI*2.0)/encoderUnitsPerRev)*(1.0/(Constants.kTalonFramePeriod*Constants.kTalonFramePeriod)));
    }

    public static double rpm_to_upf(double rpm, double encoderUnitsPerRev){
        return (rpm*encoderUnitsPerRev)*(Constants.kTalonFramePeriod/60);
    }

    public static double upf_to_rpm(int upf, double encoderUnitsPerRev){
        return (((double)upf)*(60/Constants.kTalonFramePeriod)*(1.0/encoderUnitsPerRev));
    }
}
