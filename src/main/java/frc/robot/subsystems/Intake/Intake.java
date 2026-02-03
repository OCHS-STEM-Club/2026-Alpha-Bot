// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private TalonFX rollerMotor;
  private TalonFXConfiguration rollerConfig;

  private IntakeState currentState = IntakeState.STOP;

  /** Creates a new Intake. */
  public Intake() {

  rollerMotor = new TalonFX(IntakeConstants.kIntakeMotorId);

  rollerConfig = new TalonFXConfiguration()
                      .withMotorOutput(new MotorOutputConfigs()
                                            .withInverted(InvertedValue.CounterClockwise_Positive) //Set motor inversion based on mechanism
                                            .withNeutralMode(NeutralModeValue.Brake))
                      .withCurrentLimits(new CurrentLimitsConfigs()
                                            .withSupplyCurrentLimit(IntakeConstants.kIntakeSupplyCurrentLimit));

  rollerMotor.getConfigurator().apply(rollerConfig);

  }

    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logMotorData();

  }

  public void setGoal(IntakeState desiredState) {
    currentState = desiredState;
    switch (desiredState) {
      case INTAKE:
        rollerMotor.set(IntakeConstants.kIntakeInSpeed);
        break;
      case OUTTAKE:
        rollerMotor.set(IntakeConstants.kIntakeOutSpeed);
        break;
      case STOP:
        rollerMotor.stopMotor(); //stop the rollers
        break;
    }
  }

  private void logMotorData(){
    DogLog.log("Subsystems/Intake/IntakeState", currentState.name());
    DogLog.log("Subsystems/Intake/RollerMotorVelocity", rollerMotor.getVelocity().getValueAsDouble());
    DogLog.log("Subsystems/Intake/RollerMotorSupplyCurrent", rollerMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Subsystems/Intake/RollerMotorStatorCurrent", rollerMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Subsystems/Intake/RollerMotorVoltage", rollerMotor.getMotorVoltage().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
