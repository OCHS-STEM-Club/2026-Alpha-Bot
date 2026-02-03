// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Add your docs here. */
public class ShooterConstants {
    public static int kShooterMotorId = 4;

    public static int kShooterSupplyCurrentLimit = 15;

    public static InterpolatingDoubleTreeMap kShooterMap = new InterpolatingDoubleTreeMap();

    public static void setupShooterMap(){
        kShooterMap.put(0.1, 0.5);
        kShooterMap.put(0.1, 0.5);
        kShooterMap.put(0.1, 0.5);
        kShooterMap.put(0.1, 0.5);
    }
}

