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

    public void setUpLimeLightMegaTag1() {
        LimelightHelpers.PoseEstimate mt1Result = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limeLightName);
        boolean rejectPose = processPoseRejection(mt1Result, false);

        if (!rejectPose) {
            DogLog.log("Subsystems/Vision/" + m_limeLightName + "/MT1/Raw Vision Estimate", mt1Result.pose);
            // TODO: Do we need to account for latency in the timestamp?
            // TODO: If so, we can use mt1Result.timestampSeconds - mt1Result.latency (after converting latency to seconds if necessary)
            m_swerveSubsystem.addVisionMeasurement(mt1Result.pose, mt1Result.timestampSeconds, getMegaTag1StdDevs(mt1Result));
        }
    }

    public void setUpLimeLightMegaTag2() {
        LimelightHelpers.SetRobotOrientation(m_limeLightName, m_swerveSubsystem.getState().Pose.getRotation().getDegrees(), 0, m_swerveSubsystem.getPigeon2().getPitch().getValueAsDouble(), 0, m_swerveSubsystem.getPigeon2().getRoll().getValueAsDouble(), 0);
        LimelightHelpers.PoseEstimate mt2Result = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_limeLightName);
        boolean rejectPose = processPoseRejection(mt2Result, true);
        setupLLIMUModes();

        if (!rejectPose) {
            DogLog.log("Subsystems/Vision/" + m_limeLightName + "/MT2/Raw Vision Estimate", mt2Result.pose);
            // TODO: Do we need to account for latency in the timestamp?
            // TODO: If so, we can use mt2Result.timestampSeconds - mt2Result.latency (after converting latency to seconds if necessary)
            m_swerveSubsystem.addVisionMeasurement(mt2Result.pose, mt2Result.timestampSeconds, calculateStdDevsMT2(mt2Result, mt2Result.pose));
        }
    }

    /**
     * Set up the Limelight's IMU modes based on whether the robot is disabled or enabled.
     */
    private void setupLLIMUModes() {
        LimelightHelpers.SetIMUAssistAlpha(m_limeLightName, 0.001);

        if (DriverStation.isDisabled()) {
            LimelightHelpers.SetIMUMode(m_limeLightName, 1);
        } else {
            LimelightHelpers.SetIMUMode(m_limeLightName, 4);
        }
    }

    /**
     * Process pose rejection based on various criteria for both MegaTag 1 and MegaTag 2.
     * For MegaTag 1: Reject if less than 2 tags or if any tag has high ambiguity.
     * For MegaTag 2: Reject if single tag has high ambiguity.
     * Also includes general checks for validity, latency, robot motion, and field boundaries.
     */
    private boolean processPoseRejection(LimelightHelpers.PoseEstimate poseEstimate, boolean isMT2) {
        String tagType = isMT2 ? "MT2" : "MT1";

        boolean rejectPose = 
                  !LimelightHelpers.validPoseEstimate(poseEstimate)
                || poseEstimate.tagCount == 0
                || Math.abs(m_swerveSubsystem.getState().Speeds.omegaRadiansPerSecond) > (4 * Math.PI)
                || poseEstimate.latency > VisionConstants.kMaxLatency

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
            }
        }
        return rejectPose;
    }

    /**
     * Calculate the standard deviations for the MegaTag1 pose estimation based on number of tags, average distance and average ambiguity.
     * Poses are already filtered by pose rejection, so this only scales std devs for accepted poses.
     */
    public Matrix<N3, N1> getMegaTag1StdDevs(PoseEstimate poseEstimate) {
        int numTags = poseEstimate.rawFiducials.length;
        double avgDist = 0;

        for (var fiducial : poseEstimate.rawFiducials) {
            avgDist += fiducial.distToCamera;
        }

        avgDist /= numTags;
        double avgAmbiguity = getAmbiguity(poseEstimate);

        DogLog.log("Subsystems/Vision/" + m_limeLightName + "/MT1/StdDev Factors/Num Tags", numTags);
        DogLog.log("Subsystems/Vision/" + m_limeLightName + "/MT1/StdDev Factors/Average Distance", avgDist);
        DogLog.log("Subsystems/Vision/" + m_limeLightName + "/MT1/StdDev Factors/Average Ambiguity", avgAmbiguity);

        // Start with base std devs (multi-tag assumed since single tags are rejected)
        var estStdDevs = VisionConstants.kBaseStdDevsMT1;

        // Scale by ambiguity and distance factors
        estStdDevs = estStdDevs.times((1 + avgAmbiguity) * 5);
        estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    /**
     * Calculate dynamic standard deviations for pose estimation based on multiple factors.
     */
    private Matrix<N3, N1> calculateStdDevsMT2(PoseEstimate poseEstimate, Pose2d pose) {
        int numTags = poseEstimate.rawFiducials.length;
        double avgDist = 0;

        for (var fiducial : poseEstimate.rawFiducials) {
            avgDist += fiducial.distToCamera;
        }
        avgDist /= numTags;
        // Calculate standard deviations
        double stdDevFactor = Math.pow(avgDist, 2.0) / poseEstimate.tagCount;

        double linearStdDev = 0.02 * stdDevFactor;

        linearStdDev *= 0.5;

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