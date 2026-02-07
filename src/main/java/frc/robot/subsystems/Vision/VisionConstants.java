
package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {

    public static final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // Rejection thresholds
    public static final double kMaxAmbiguity = 0.3;
    public static final double kMaxLatency = 20; // milliseconds

    // Base standard deviations for MT1 (multi-tag)
    public static final Matrix<N3, N1> kBaseStdDevsMT1 = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, 3);
    //TODO: Check how the last one works, since it's an angle - maybe should be in radians and smaller?

    public static final double linearStdDevBaseline = 0.09; // meters
    public static final double linearStdDevMegatag2Factor = 0.05; // meters per meter distance

    public static final double angularStdDevBaseline = Units.degreesToRadians(4); // radians


  }