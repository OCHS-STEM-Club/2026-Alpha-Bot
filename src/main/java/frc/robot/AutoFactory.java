package frc.robot;

import static edu.wpi.first.units.Units.Rotation;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
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

    private PIDController translationController = new PIDController(1.5, 0.0, 0.0);
    private PIDController rotationController = new PIDController(2.5, 0.0, 0.0);
    private PIDController crossTrackController = new PIDController(1, 0.0, 0.0);
    

    private ApplyRobotSpeeds m_driveRequest = new ApplyRobotSpeeds();
    // .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    // .withDesaturateWheelSpeeds(true);

    private FollowPath.Builder pathBuilder;

    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public AutoFactory(CommandSwerveDrivetrain swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;

        pathBuilder = new FollowPath.Builder(
                m_swerveSubsystem,
                () -> m_swerveSubsystem.getState().Pose,
                () -> m_swerveSubsystem.getState().Speeds,
                (speeds) -> m_swerveSubsystem.setControl(m_driveRequest.withSpeeds(speeds)),
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

        FollowPath.setPoseLoggingConsumer(pair -> {
            Logger.recordOutput(pair.getFirst(), pair.getSecond());
        });

        FollowPath.setTranslationListLoggingConsumer(pair -> {
            Logger.recordOutput(pair.getFirst(), pair.getSecond());
        });

        FollowPath.setDoubleLoggingConsumer(pair -> {
            Logger.recordOutput(pair.getFirst(), pair.getSecond());
        });
        
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
