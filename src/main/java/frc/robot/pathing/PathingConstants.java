package frc.robot.pathing;

import edu.wpi.first.math.util.Units;

public class PathingConstants {
    public static final double kP_X = 0.1;
    public static final double kI_X = 0;
    public static final double kD_X = 0;

    public static final double kP_Y = 0.1;
    public static final double kI_Y = 0;
    public static final double kD_Y = 0;

    public static final double kP_THETA = 0.0075;

    public static final double kErrorTolerance_X = Units.inchesToMeters(10);
    public static final double kErrorTolerance_Y = Units.inchesToMeters(10);

    public static final double kErrorToleranceDerivative_X = 0.2;
    public static final double kErrorToleranceDerivative_Y = 0.1;

    public static final double kPositionTolerance_Theta = 4; // degrees
    public static final double kVelocityTolerance_Theta = 0.5; // radians per second

    public static final double kMaxVelocity_X = 2.5; // meters per second
    public static final double kMinVelocity_X = -2.5; // meters per second

    public static final double kMaxVelocity_Y = 2.5; // meters per second
    public static final double kMinVelocity_Y = -2.5; // meters per second
  }
