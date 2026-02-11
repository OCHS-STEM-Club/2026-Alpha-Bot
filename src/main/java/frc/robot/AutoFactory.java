package frc.robot;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class AutoFactory extends SubsystemBase{

    private CommandSwerveDrivetrain m_swerveSubsystem;

    private PIDController translationController = new PIDController(0.2, 0.0, 0.0);
    private PIDController rotationController = new PIDController(0.3, 0.0, 0.0);
    private PIDController crossTrackController = new PIDController(0.0, 0.0, 0.0);
    

    private SwerveRequest.ApplyRobotSpeeds m_driveRequest = new ApplyRobotSpeeds()
    .withDriveRequestType(DriveRequestType.Velocity);

    private FollowPath.Builder pathBuilder;

    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public AutoFactory(CommandSwerveDrivetrain swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;

        pathBuilder = new FollowPath.Builder(
                m_swerveSubsystem,
                () -> m_swerveSubsystem.getState().Pose,
                () -> m_swerveSubsystem.getState().Speeds,
                speeds -> m_swerveSubsystem.setControl(m_driveRequest.withSpeeds(speeds)),
                translationController,
                rotationController,
                crossTrackController
            ).withDefaultShouldFlip();
    }


    public Command getStraightAuto(){
        Path firstPath = new Path("StraightPath");
        Rotation2d initialDirection = firstPath.getInitialModuleDirection();

        m_swerveSubsystem.applyRequest(() ->
            point.withModuleDirection(initialDirection));
        
        return Commands.sequence(
            pathBuilder.build(firstPath)
        );
    }

    public Command getTranslationTuningAuto(){
        Path TranslationTuningPath = new Path("TranslationTuning");
        Rotation2d initialDirection = TranslationTuningPath.getInitialModuleDirection();

        m_swerveSubsystem.applyRequest(() ->
            point.withModuleDirection(initialDirection));
        
        return Commands.sequence(
            pathBuilder.build(TranslationTuningPath)
        );
    }

    public Command getRotationTuningAuto(){
        Path RotationTuningPath = new Path("RotationTuning");
        Rotation2d initialDirection = RotationTuningPath.getInitialModuleDirection();

        m_swerveSubsystem.applyRequest(() ->
            point.withModuleDirection(initialDirection));
        
        return Commands.sequence(
            pathBuilder.build(RotationTuningPath)
        );
    }

    public Command getNeutralAuto(){
        Path RotationTuningPath = new Path("neutral");
        Rotation2d initialDirection = RotationTuningPath.getInitialModuleDirection();

        m_swerveSubsystem.applyRequest(() ->
            point.withModuleDirection(initialDirection));
        
        return Commands.sequence(
            pathBuilder.build(RotationTuningPath)
        );
    }



    @Override
    public void periodic(){
        SmartDashboard.putData("Auto Translation Controller", translationController);
        SmartDashboard.putData("Auto Rotation Controller", rotationController);
        SmartDashboard.putData("Auto Cross Track Controller", crossTrackController);
    }
}
