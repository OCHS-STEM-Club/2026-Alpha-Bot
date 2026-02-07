package frc.robot.subsystems.Vision;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class Vision {
    private CommandSwerveDrivetrain m_swerveSubsystem;
    private String m_limeLightName;

    public Vision(CommandSwerveDrivetrain swerveSubsystem, String limeLightName) {
        m_swerveSubsystem = swerveSubsystem;
        m_limeLightName = limeLightName;
    }

    /**
     * Process MegaTag1 vision updates and add valid measurements to the drivetrain.
     * MT1 requires at least 2 tags for pose estimation.
     */
    public void updateMegaTag1() {
        PoseEstimate mt1Result = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limeLightName);

        if (shouldRejectPose(mt1Result, false)) {
            return;
        }

        DogLog.log("Subsystems/Vision/" + m_limeLightName + "/MT1/Raw Vision Estimate", mt1Result.pose);
        m_swerveSubsystem.addVisionMeasurement(
            mt1Result.pose,
            mt1Result.timestampSeconds,
            getMegaTag1StdDevs(mt1Result)
        );
    }

    /**
     * Process MegaTag2 vision updates and add valid measurements to the drivetrain.
     * MT2 uses robot orientation from the gyro for more accurate pose estimation.
     */
    public void updateMegaTag2() {

        LimelightHelpers.SetRobotOrientation(
            m_limeLightName,
            m_swerveSubsystem.getState().Pose.getRotation().getDegrees(),
            m_swerveSubsystem.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(),
            m_swerveSubsystem.getPigeon2().getPitch().getValueAsDouble(),
            0,
            m_swerveSubsystem.getPigeon2().getRoll().getValueAsDouble(),
            0
        );

        PoseEstimate mt2Result = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_limeLightName);

        if (shouldRejectPose(mt2Result, true)) {
            return;
        }

        DogLog.log("Subsystems/Vision/" + m_limeLightName + "/MT2/Raw Vision Estimate", mt2Result.pose);
        m_swerveSubsystem.addVisionMeasurement(
            mt2Result.pose,
            mt2Result.timestampSeconds,
            calculateStdDevsMT2(mt2Result, mt2Result.pose)
        );
    }


    /**
     * Determine if a pose estimate should be rejected based on validity criteria.
     * For MegaTag 1: Reject if less than 2 tags or if any tag has high ambiguity.
     * For MegaTag 2: Reject if single tag has high ambiguity.
     * Also includes general checks for validity, latency, robot motion, and field boundaries.
     */
    private boolean shouldRejectPose(PoseEstimate poseEstimate, boolean isMT2) {
        String tagType = isMT2 ? "MT2" : "MT1";

        boolean rejectPose = 
                  !LimelightHelpers.validPoseEstimate(poseEstimate)
                || poseEstimate.tagCount == 0
                || Math.abs(m_swerveSubsystem.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 50 // degrees per second - reject if spinning too fast
                // || poseEstimate.latency > VisionConstants.kMaxLatency

                || Math.abs(m_swerveSubsystem.getPigeon2().getPitch().getValueAsDouble()) > 10 // degrees - reject if over bump
                || Math.abs(m_swerveSubsystem.getPigeon2().getRoll().getValueAsDouble()) > 10 // degrees - reject if tilted too much

                || poseEstimate.pose.getX() < 0.0
                || poseEstimate.pose.getX() > VisionConstants.aprilTagLayout.getFieldLength()
                || poseEstimate.pose.getY() < 0.0
                || poseEstimate.pose.getY() > VisionConstants.aprilTagLayout.getFieldWidth();

        if (!rejectPose) {
            for (var estimate : poseEstimate.rawFiducials) {
                if (isMT2) {
                    if (poseEstimate.tagCount == 1 && estimate.ambiguity > VisionConstants.kMaxAmbiguity) {
                        rejectPose = true;
                        break;
                    }
                } else {
                    if (poseEstimate.tagCount < 2) {
                        rejectPose = true;
                        break;
                    } else if (getAmbiguity(poseEstimate) > VisionConstants.kMaxAmbiguity) {
                        // Reject only if average ambiguity is too high
                        rejectPose = true;
                        break;
                    }
                }

                DogLog.log("Subsystems/Vision/" + m_limeLightName + "/" + tagType + "/Rejection Factors/Ambiguity", getAmbiguity(poseEstimate));
                DogLog.log("Subsystems/Vision/" + m_limeLightName + "/" + tagType + "/Rejection Factors/Latency", poseEstimate.latency);
                DogLog.log("Subsystems/Vision/" + m_limeLightName + "/" + tagType + "/Rejection Factors/Pitch", m_swerveSubsystem.getPigeon2().getPitch().getValueAsDouble());
                DogLog.log("Subsystems/Vision/" + m_limeLightName + "/" + tagType + "/Rejection Factors/Roll", m_swerveSubsystem.getPigeon2().getRoll().getValueAsDouble());
            }
        }
        return rejectPose;
    }

    // /**
    //  * Calculate the standard deviations for the MegaTag1 pose estimation based on number of tags, average distance and average ambiguity.
    //  * Poses are already filtered by pose rejection, so this only scales std devs for accepted poses.
    //  */
    // public Matrix<N3, N1> getMegaTag1StdDevs(PoseEstimate poseEstimate) {

    //     double avgDist = poseEstimate.avgTagDist;
    //     double avgAmbiguity = getAmbiguity(poseEstimate);

    //     DogLog.log("Subsystems/Vision/" + m_limeLightName + "/MT1/StdDev Factors/Num Tags", poseEstimate.tagCount);
    //     DogLog.log("Subsystems/Vision/" + m_limeLightName + "/MT1/StdDev Factors/Average Distance", avgDist);
    //     DogLog.log("Subsystems/Vision/" + m_limeLightName + "/MT1/StdDev Factors/Average Ambiguity", avgAmbiguity);

    //     // Start with base std devs (multi-tag assumed since single tags are rejected)
    //     var estStdDevs = VisionConstants.kBaseStdDevsMT1;

    //     // Scale by ambiguity and distance factors
    //     estStdDevs = estStdDevs.times((1 + avgAmbiguity) * 5);
    //     estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    //     return estStdDevs;
    // }

        /**
     * Calculate the standard deviations for the MegaTag1 pose estimation based on number of tags, average distance and average ambiguity.
     * Poses are already filtered by pose rejection, so this only scales std devs for accepted poses.
     */
    public Matrix<N3, N1> getMegaTag1StdDevs(PoseEstimate poseEstimate) {

        double avgDist = poseEstimate.avgTagDist;
        // Calculate standard deviations
        double stdDevFactor = Math.pow(avgDist, 2.0) / poseEstimate.tagCount;

        double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor;


        return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, angularStdDev);
    }

    /**
     * Calculate dynamic standard deviations for pose estimation based on multiple factors.
     */
    private Matrix<N3, N1> calculateStdDevsMT2(PoseEstimate poseEstimate, Pose2d pose) {

        double avgDist = poseEstimate.avgTagDist;
        // Calculate standard deviations
        double stdDevFactor = Math.pow(avgDist, 2.0) / poseEstimate.tagCount;

        double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor;

        linearStdDev *= VisionConstants.linearStdDevMegatag2Factor;

        return VecBuilder.fill(linearStdDev, linearStdDev, Double.MAX_VALUE);
        
    }

    /**
     * Calculate average ambiguity from Limelight pose estimate.
     */
    private double getAmbiguity(PoseEstimate poseEstimate) {
        double totalAmbiguity = 0;
        for (var fiducial : poseEstimate.rawFiducials) {
            totalAmbiguity += fiducial.ambiguity;
        }
        return totalAmbiguity / poseEstimate.rawFiducials.length;
    }

}