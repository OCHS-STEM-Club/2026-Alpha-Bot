
package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionConstants {

    public static final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // Rejection thresholds
    public static final double kMaxAmbiguity = 0.2; // Maximum acceptable ambiguity for pose estimates
    public static final double kMaxLatency = 20; // Maximum acceptable latency for pose estimates in seconds
    // TODO: Tune these values based on testing

    // Base standard deviations for MT1 (multi-tag)
    public static final Matrix<N3, N1> kBaseStdDevsMT1 = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, 3);
    //TODO: Check how the last one works, since it's an angle - maybe should be in radians and smaller?

    // Dynamic standard deviation calculation constants for MT2
    public static final Matrix<N3, N1> kBaseStdDevsMT2 = VecBuilder.fill(0.7, 0.7, Double.MAX_VALUE);

    public static final double kDistanceMultiplier = 5.0;
    public static final double kNoisyDistance = 0.8; // meters - distance beyond which noise increases significantly

    public static final double kAmbiguityMultiplier = 0.4;
    public static final double kAmbiguityShifter = 0.2;

    public static final double kTargetMultiplier = 80.0; // Reduction per additional tag

    public static final double kLatencyMultiplier = 1.3;

  }