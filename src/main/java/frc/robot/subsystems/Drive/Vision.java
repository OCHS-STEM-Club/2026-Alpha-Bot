package frc.robot.subsystems.Drive;


import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Vision {
    private CommandSwerveDrivetrain m_swerveSubsystem;
    private String m_limeLightName;

    // MegaTag 1 Std Devs - shouldnt change too much based on the year
    public static final Matrix<N3, N1> kSingleTagStdDevsMT1 = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, 4);
    public static final Matrix<N3, N1> kMultiTagStdDevsMT1 = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, 3);

    //will need to change based on year
    public static final Matrix<N3, N1> kReefStdDevs = VecBuilder.fill(0.01, 0.01, Double.MAX_VALUE);

    public Vision(CommandSwerveDrivetrain swerveSubsystem, String limeLightName) {
        m_swerveSubsystem = swerveSubsystem;
        m_limeLightName = limeLightName;
        
        
    }

    public void setUpLimeLightMegaTag1() {
        boolean mt1ValidPose = true;
    
        LimelightHelpers.PoseEstimate mt1Result = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limeLightName);
    
        if(mt1Result != null) {
          if (mt1Result.tagCount == 0) {
            mt1ValidPose = false;
          }
    
          DogLog.log("Subsystems/VisionSubsystem/MegaTag1/Pose2D", mt1Result.pose);
    
          if (mt1ValidPose) {
            m_swerveSubsystem.addVisionMeasurement(mt1Result.pose, mt1Result.timestampSeconds, getMegaTag1StdDevs(mt1Result));
          }
        }
    }

    public void setUpLimeLightMegaTag2() {
        boolean mt2ValidPose = true;
    
        LimelightHelpers.SetRobotOrientation(m_limeLightName, m_swerveSubsystem.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    
        LimelightHelpers.PoseEstimate mt2Result = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_limeLightName);
    
        if(mt2Result != null) {
          if (mt2Result.tagCount == 0 || Math.abs(m_swerveSubsystem.getState().Speeds.omegaRadiansPerSecond) > (4 * Math.PI )) {
            mt2ValidPose = false;
          }
    
          DogLog.log("Subsystems/VisionSubsystem/MegaTag2/Pose2D", mt2Result.pose);
    
          if (mt2ValidPose) {
            m_swerveSubsystem.addVisionMeasurement(mt2Result.pose, mt2Result.timestampSeconds, getEstimationStdDevsLimelightMT2(mt2Result));
          }
        }
    }



    // Calculate the standard deviations for the MegaTag1 pose estimation based on number of tags, average distance and average ambiguity
  public Matrix<N3, N1> getMegaTag1StdDevs(PoseEstimate poseEstimate){
    var estStdDevs = kSingleTagStdDevsMT1;

    // Calculate the number of tags, average distance and average ambiguity
    int numTags = 0; 
    double avgDist = 0;
    double avgAmbiguity = 0;
    for(var value : poseEstimate.rawFiducials){ // Loop through all the tags detected
      numTags++;
      avgDist += value.distToCamera;
      avgAmbiguity += value.ambiguity;
    }

    // if no tags detected return single tag std devs
    if (numTags == 0) {
      return estStdDevs;
    }

    // Calculate the averages
    avgDist /= numTags;
    avgAmbiguity /= numTags;

    // Adjust the standard deviations based on number of tags
    if (numTags > 1) {
      estStdDevs = kMultiTagStdDevsMT1;
    }
    // If the average ambiguity is too high, return very high std devs to ignore the pose
    if (avgAmbiguity > 0.7) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    // Scale the standard deviations based on the average ambiguity
    estStdDevs.times((1 + avgAmbiguity) * 5);

    // Log the values
    DogLog.log("Subsystems/VisionSubsystem/MegaTag1/Average Ambiguity", avgAmbiguity);
    DogLog.log("Subsystems/VisionSubsystem/MegaTag1/Num Tags", numTags);
    DogLog.log("Subsystems/VisionSubsystem/MegaTag1/Average Distance", avgDist);

    // If the average distance is too far, return very high std devs to ignore the pose
    if (numTags == 1 && avgDist > 1.5) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }else{ // Scale the standard deviations based on the average distance
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist/30));
    }

    return estStdDevs;
  }

  public static Matrix<N3, N1> getEstimationStdDevsLimelightMT2(PoseEstimate poseEstimate) {
    var estStdDevs = kReefStdDevs;
    
    int numTags = 0;
    double avgDist = 0;
    for (var value : poseEstimate.rawFiducials) {
        numTags++;
        avgDist += value.distToCamera;
    }

    if (numTags == 0) {
        return estStdDevs;
    }

    avgDist /= numTags;

    // Decrease std devs if multiple targets are visible
    if (numTags > 1) {
        estStdDevs.times(0.7);
    }

    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 5) {
        estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }else {
        estStdDevs = estStdDevs.times(1 + (avgDist * avgDist * 5));
    }

    return estStdDevs;
}



}