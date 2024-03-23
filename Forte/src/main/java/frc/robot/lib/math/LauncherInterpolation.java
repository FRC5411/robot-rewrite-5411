package frc.robot.lib.math;

import frc.robot.lib.Interpolation.InterpolatingDouble;
import frc.robot.lib.Interpolation.InterpolatingTreeMap;

public class LauncherInterpolation {
    

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> launcherMap = new InterpolatingTreeMap<>();

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> pivotMap = new InterpolatingTreeMap<>();
    // private static final double PIGEON_OFFSET = 180;

// key = distance from speaker meters
// value = rpm setpoint
    static {    
        pivotMap.put(new InterpolatingDouble(1.44), new InterpolatingDouble(57.0));
        pivotMap.put(new InterpolatingDouble(1.85), new InterpolatingDouble(49.0));
        pivotMap.put(new InterpolatingDouble(2.207), new InterpolatingDouble(42.5));
        pivotMap.put(new InterpolatingDouble(2.61), new InterpolatingDouble(36.0));
        pivotMap.put(new InterpolatingDouble(2.72), new InterpolatingDouble(35.0));
        pivotMap.put(new InterpolatingDouble(3.06), new InterpolatingDouble(32.0));
        pivotMap.put(new InterpolatingDouble(3.33), new InterpolatingDouble(28.0));
        pivotMap.put(new InterpolatingDouble(3.59), new InterpolatingDouble(26.5));
        pivotMap.put(new InterpolatingDouble(3.8), new InterpolatingDouble(25.0));
        pivotMap.put(new InterpolatingDouble(4.01), new InterpolatingDouble(24.2));
        pivotMap.put(new InterpolatingDouble(4.22), new InterpolatingDouble(23.4));
        pivotMap.put(new InterpolatingDouble(4.41), new InterpolatingDouble(22.8));
        pivotMap.put(new InterpolatingDouble(4.6), new InterpolatingDouble(22.8));
        pivotMap.put(new InterpolatingDouble(4.7), new InterpolatingDouble(22.3));
        pivotMap.put(new InterpolatingDouble(5.1), new InterpolatingDouble(22.1));
        pivotMap.put(new InterpolatingDouble(5.2), new InterpolatingDouble(22.0));
        pivotMap.put(new InterpolatingDouble(5.3), new InterpolatingDouble(21.7));
    }
}
