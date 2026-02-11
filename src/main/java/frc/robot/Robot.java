// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.util.LimelightHelpers;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

        // if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        // } else {
        //     setUseTiming(false); // Run as fast as possible
        //     String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        //     Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        //     Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        // }

        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        LimelightHelpers.SetIMUMode("limelight-front", 1); //seeding
        LimelightHelpers.SetIMUMode("limelight-left", 1); //seeding
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
        if(m_robotContainer.drivetrain.notRotating()){
            LimelightHelpers.SetIMUMode("limelight-front", 1); //seeding
            LimelightHelpers.SetIMUMode("limelight-left", 1); //seeding
        }else{
            LimelightHelpers.SetIMUMode("limelight-front", 2); 
            LimelightHelpers.SetIMUMode("limelight-left", 2); 
        }
    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
        if(m_robotContainer.drivetrain.notRotating()){
            LimelightHelpers.SetIMUMode("limelight-front", 1); //seeding
            LimelightHelpers.SetIMUMode("limelight-left", 1); //seeding
        }else{
            LimelightHelpers.SetIMUMode("limelight-front", 2); 
            LimelightHelpers.SetIMUMode("limelight-left", 2); 
        }
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
