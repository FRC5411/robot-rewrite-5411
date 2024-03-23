package frc.robot.lib.util.math;

public class Conversions {

    public static double neoToMeters(double positionCounts, double circumference, double gearRatio){
        return positionCounts * (circumference / (gearRatio * 42.0));
    }

    public static double absoluteEncoderToDegrees(double speedMetersPerSecond, double wheelcircumference,
            double drivegearratio) {
        return speedMetersPerSecond * (360.0 / (wheelcircumference * 4096.0));//If does not work, leave blank and add TODO
    }

    public static double degreesToCANSparkMax(double degrees, double angleGearRatio) {
        return degrees / (360.0 / (angleGearRatio * 42)); //If does not work, leave blank and add TODO
    }
} 