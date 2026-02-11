package frc.robot.pathing;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;;

public class PathingFactory extends SubsystemBase{

    private final CommandSwerveDrivetrain m_SwerveSubsystem;

    private SwerveRequest.FieldCentricFacingAngle alignAngleRequest = new FieldCentricFacingAngle()
    .withDriveRequestType(DriveRequestType.Velocity)
    .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    public PIDController xController = new PIDController(
        PathingConstants.kP_X,
        PathingConstants.kI_X,
        PathingConstants.kD_X);
    public PIDController yController = new PIDController(
        PathingConstants.kP_Y,
        PathingConstants.kI_Y,
        PathingConstants.kD_Y); 



    public PathingFactory(CommandSwerveDrivetrain swerveSubsystem) {
        this.m_SwerveSubsystem = swerveSubsystem;
    }


    public Command driveToPose(Pose2d targetPose){
        alignAngleRequest.HeadingController.setP(PathingConstants.kP_THETA);
        alignAngleRequest.HeadingController.setTolerance(Units.degreesToRadians(PathingConstants.kPositionTolerance_Theta), Units.degreesToRadians(PathingConstants.kVelocityTolerance_Theta));

        xController.setTolerance(PathingConstants.kErrorTolerance_X);
        yController.setTolerance(PathingConstants.kErrorTolerance_Y);


        return m_SwerveSubsystem.applyRequest(()-> {
            Pose2d currentPose = m_SwerveSubsystem.getState().Pose;

            double xVelocity = MathUtil.clamp(xController.calculate(currentPose.getX(), targetPose.getX()), PathingConstants.kMinVelocity_X, PathingConstants.kMaxVelocity_X);
            double yVelocity = MathUtil.clamp(yController.calculate(currentPose.getY(), targetPose.getY()), PathingConstants.kMinVelocity_Y, PathingConstants.kMaxVelocity_Y);

            DogLog.log("Pathing/DriveToPose/TargetPose", targetPose);

            return 
                alignAngleRequest.withTargetDirection(targetPose.getRotation())
                .withVelocityX(xVelocity)
                .withVelocityY(yVelocity);
                }).until(() -> 
                        xController.atSetpoint() && 
                        yController.atSetpoint() && 
                        alignAngleRequest.HeadingController.atSetpoint());

        
                
    }


    public Command driveToPoseWithTimeout(Pose2d targetPose, double timeout){
        alignAngleRequest.HeadingController.setP(PathingConstants.kP_THETA);
        alignAngleRequest.HeadingController.setTolerance(Units.degreesToRadians(PathingConstants.kPositionTolerance_Theta), Units.degreesToRadians(PathingConstants.kVelocityTolerance_Theta));

        xController.setTolerance(PathingConstants.kErrorTolerance_X, PathingConstants.kErrorToleranceDerivative_X);
        yController.setTolerance(PathingConstants.kErrorTolerance_Y, PathingConstants.kErrorToleranceDerivative_Y);
  
        return m_SwerveSubsystem.applyRequest(()-> {
            Pose2d currentPose = m_SwerveSubsystem.getState().Pose;

            double xVelocity = MathUtil.clamp(xController.calculate(currentPose.getX(), targetPose.getX()), PathingConstants.kMinVelocity_X, PathingConstants.kMaxVelocity_X);
            double yVelocity = MathUtil.clamp(yController.calculate(currentPose.getY(), targetPose.getY()), PathingConstants.kMinVelocity_Y, PathingConstants.kMaxVelocity_Y);


            return 
                alignAngleRequest.withTargetDirection(targetPose.getRotation())
                .withVelocityX(xVelocity)
                .withVelocityY(yVelocity);
                }).until(() -> 
                        xController.atSetpoint() && 
                        yController.atSetpoint() && 
                        alignAngleRequest.HeadingController.atSetpoint())
                .withTimeout(timeout);
        
                
    }

    @Override
    public void periodic(){
        SmartDashboard.putData( "X Pose Controller", xController);
        SmartDashboard.putData("Y Pose Controller", yController);
    }




}
