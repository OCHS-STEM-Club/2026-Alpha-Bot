// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import dev.doglog.DogLog;

public class Vision extends SubsystemBase {

  private final CommandSwerveDrivetrain m_drivetrain; //change back to drivetrain if errors occur

  // Camera configuration
  private static final String[] CAMERAS = {"limelight-front", "limelight-back"};

  // 2026 REBUILT field layout for bounds checking
  private static final AprilTagFieldLayout FIELD_LAYOUT = 
          AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  
  // Connection tracking
  private final Alert[] disconnectedAlerts = new Alert[CAMERAS.length];
  private final long[] lastUpdateTimestamps = new long[CAMERAS.length];
  private static final long CONNECTION_TIMEOUT_US = 250_000; //250ms

  // Rejection thresholds
  private static final double MAX_ANGULAR_VELOCITY = 360.0; //deg/s - reject MT2 while spinning
  private static final double MAX_AMBIGUITY = 0.3;          //reject ambiguous single-tag poses
  private static final double MT2_MAX_DISTANCE = 5.0;       //meters
  private static final double MT1_MAX_DISTANCE = 7.0;       //meters (rotation valid at longer range)

  // Standard deviation bases (scaled by distance and tag count)
  private static final double MT2_XY_STD_DEV = 0.5;         //meters
  private static final double MT1_ROTATION_STD_DEV = 2.0;   //radians

  public Vision(CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;

    for (int i = 0; i < CAMERAS.length; i++) {
      disconnectedAlerts[i] = new Alert(CAMERAS[i] + " disconnected", AlertType.kError);
    }
  }

  @Override
  public void periodic() {
    var state = m_drivetrain.getState();
    double yaw = state.Pose.getRotation().getDegrees();
    boolean spinning = Math.toDegrees(Math.abs(state.Speeds.omegaRadiansPerSecond)) > MAX_ANGULAR_VELOCITY;

    for (int i = 0; i < CAMERAS.length; i++) {
      String cam = CAMERAS[i];

      // Check connection and update driver station alert
      boolean connected = updateConnection(cam, i);
      disconnectedAlerts[i].set(!connected);
      if (!connected) continue;

      // Configure Limelight IMU mode and send robot heading
      LimelightHelpers.SetIMUMode(cam, DriverStation.isEnabled() ? 4 : 1);
      LimelightHelpers.SetRobotOrientation_NoFlush(cam, yaw, 0, 0, 0, 0, 0);

      // Process pose estimates
      processMT2(cam, spinning);
      processMT1(cam);
    }

    // Batch flush all Limelight updates
    LimelightHelpers.Flush();
    DogLog.log("Vision/Connected", new boolean[]{isConnected(0), isConnected(1)});
  }

  // ====================== CONNECTION ========================

  private boolean updateConnection(String cam, int idx) {
    long now = RobotController.getFPGATime();

    if (LimelightHelpers.getLatency_Pipeline(cam) > 0) {
      lastUpdateTimestamps[idx] = now;
    }

    return (now - lastUpdateTimestamps[idx]) < CONNECTION_TIMEOUT_US;
  }

  private boolean isConnected(int idx) {
    return (RobotController.getFPGATime() - lastUpdateTimestamps[idx]) < CONNECTION_TIMEOUT_US;
  }

  // ==================== POSE PROCESSING ======================

  private void processMT2(String cam, boolean spinning) {
    PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cam);

    if (!isValidEstimate(est, MT2_MAX_DISTANCE) || spinning) return;

    // Caluclate std dev: base * multi-tag bonus * distance scaling
    double stdDev = MT2_XY_STD_DEV
        * (est.tagCount > 1 ? 0.7 : 1.0)
        * (1.0 + est.avgTagDist * est.avgTagDist / 30.0);

    m_drivetrain.addVisionMeasurement(
      est.pose,
      Utils.fpgaToCurrentTime(est.timestampSeconds),
      VecBuilder.fill(stdDev, stdDev, Double.MAX_VALUE) //trust X/Y, ignore rotation
    );

    DogLog.log("Vision/" + cam + "/MT2/Pose", est.pose);
    DogLog.log("Vision/" + cam + "/MT2/TagCount", est.tagCount);
  }

  /*
   * Process Megatag1 for rotation estimation.
   * Only used when seeing 2+ tags (single-tag MT1 has ambiguity issues).
   * 
   */
  private void processMT1(String cam) {
    PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue(cam);

    // Calculate std dev: base * distance scaling
    double stdDev = MT1_ROTATION_STD_DEV
      * (1.0 + est.avgTagDist * est.avgTagDist / 30.0);

    m_drivetrain.addVisionMeasurement(
      est.pose,
      Utils.fpgaToCurrentTime(est.timestampSeconds),
      VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, stdDev) // ignore X/Y, trust rotation
    );

    DogLog.log("Vision/" + cam + "/MT1/Pose", est.pose);
  }

  // ====================== VALIDATION ======================

  /*
   * Validates a pose estimate by checking:
   * - valid data from Limelight
   * - Within max distance threshold
   * - Low ambiguity (for single-tag)
   * - Inside field boundaries
   */
  private boolean isValidEstimate(PoseEstimate est, double maxDist) {
    if (!LimelightHelpers.validPoseEstimate(est)) return false;
    if (est.avgTagDist > maxDist) return false;
    if (est.tagCount == 1 && est.rawFiducials.length > 0 && est.rawFiducials[0].ambiguity > MAX_AMBIGUITY) return false;
    return !isOutsideField(est.pose);
  }

  /*
   * Checks if pose is outside field boundaries (with 0.5m margin for measurement error).
   */
  private boolean isOutsideField(Pose2d pose) {
    double x = pose.getX();
    double y = pose.getY();

    return x < -0.5 || x > FIELD_LAYOUT.getFieldLength() + 0.5
      || y < -0.5 || y > FIELD_LAYOUT.getFieldWidth() + 0.5;
  }

  // ======================= PUBLIC API ======================

  // Returns true if either camera can currently see an AprilTag.
  public boolean canSeeTag() {
     for (int i = 0; i < CAMERAS.length; i++) {
        if (isConnected(i) && LimelightHelpers.getTV(CAMERAS[i])) return true;
     }
     return false;
  }

  // returns true if front Limelight is connected.
  public boolean isFrontConnected() {
    return isConnected(0);
  }

  //Returns true if back Limelight is connected.
  public boolean isBackConnected(){
    return isConnected(1);
  }
}
