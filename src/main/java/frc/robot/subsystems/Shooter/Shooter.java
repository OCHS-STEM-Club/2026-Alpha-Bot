// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.security.PublicKey;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterState;


public class Shooter extends SubsystemBase {
  private TalonFX shooterMotor;
  private TalonFXConfiguration shooterConfig;

  private MotionMagicVelocityVoltage m_motionRequest;

  private VoltageOut m_voltageRequest;

  private ShooterState currentState = ShooterState.STOP;
  /** Creates a new Shooter. */
  public Shooter() {
    shooterMotor = new TalonFX(ShooterConstants.kShooterMotorId);

    shooterConfig = new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                              .withNeutralMode(NeutralModeValue.Brake)
                                              .withInverted(InvertedValue.Clockwise_Positive))
                        .withSlot0(new Slot0Configs()
                                        .withKP(1)
                                        .withKI(1)
                                        .withKD(1))
                        .withMotionMagic(new MotionMagicConfigs()
                                              .withMotionMagicCruiseVelocity(1000)
                                              .withMotionMagicAcceleration(1000)
                                              .withMotionMagicJerk(1000))
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                              .withSupplyCurrentLimit(ShooterConstants.kShooterSupplyCurrentLimit));

    shooterMotor.getConfigurator().apply(shooterConfig);

    m_voltageRequest = new VoltageOut(0);

    m_motionRequest = new MotionMagicVelocityVoltage(0).withSlot(0).withFeedForward(0);

    ShooterConstants.setupShooterMap();
  }

  public void setGoal(ShooterState desiredState) {
    switch (desiredState) {
      case BLUE_HUB:
        shooterMotor.set(ShooterConstants.kShooterMap.get(0.1));
        break;
      case BLUE_OUTPOST_SHUTTLING:
        shooterMotor.set(ShooterConstants.kShooterMap.get(0.2));
        break;
      case BLUE_DEPOT_SHUTTILING:
        shooterMotor.set(ShooterConstants.kShooterMap.get(0.3));
        break;
      case RED_HUB:
        shooterMotor.set(ShooterConstants.kShooterMap.get(0.4));
        break;
      case RED_OUTPOST_SHUTTLING:
        shooterMotor.set(ShooterConstants.kShooterMap.get(0.5));
        break;
      case RED_DEPOT_SHUTTLING:
        shooterMotor.set(ShooterConstants.kShooterMap.get(0.6));
        break;
      case STOP:
        shooterMotor.stopMotor();
        break;
    }
  }
  
  public void shooterOn() {
    shooterMotor.set(0.5);
  }

  public void shooterReverse() {
    shooterMotor.set(-0.5);
  }

  public void shooterStop() {
    shooterMotor.set(0);
  }

  /*private final SysIdRoutine m_sysIdRoutine =
    new SysIdRoutine(
      new SysIdRoutine.Config(
        null,
        Volts.of(4),
        Seconds.of(10),
        (state) -> SignalLogger.writeString("End Effector State", state.toString())
      ), 
      new SysIdRoutine.Mechanism(null, null, null));*/

  // subject to change
  public void setShooterVelocity(double velocity) {
    shooterMotor.setControl(m_motionRequest.withVelocity(velocity));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logMotorData();
  }

  private void logMotorData() {
    DogLog.log("Subsystems/Shooter/ShooterState", currentState.name());
    DogLog.log("Subsystems/Shooter/ShooterMotorVelocity", shooterMotor.getVelocity().getValueAsDouble());
    //DogLog.log("Subsystems/Shooter/ShooterSetpoint", shooterMotor.getPosition().getValueAsDouble()); //subject to change
    //DogLog.log("Subsystems/Shooter/IsAtSetpoint", );
    DogLog.log("Subsystems/Shooter/ShooterMotorSupplyCurrent", shooterMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Subsystems/Shooter/ShooterMotorStatorCurrent", shooterMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Subsystems/Shooter/ShooterMotorVoltage", shooterMotor.getMotorVoltage().getValueAsDouble());
  }
}
