// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PoseConstants;
import frc.robot.pathing.PathingFactory;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto extends SequentialCommandGroup {
  /** Creates a new TestAuto. */
  private PathingFactory m_pathingFactory;
  private CommandSwerveDrivetrain m_swerveSubsystem;
  public TestAuto(PathingFactory pathingFactory, CommandSwerveDrivetrain swerveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.m_pathingFactory = pathingFactory;
    this.m_swerveSubsystem = swerveSubsystem;
    addCommands(
      m_pathingFactory.driveToPose(new Pose2d(1.965, 1.2, Rotation2d.fromDegrees(0))) 
    );
  }
}
