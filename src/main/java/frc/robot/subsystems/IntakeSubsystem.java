// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax m_rollerMotor;
  private final CANSparkMax m_intakeMotor;
  private final RelativeEncoder m_intakeMotorEncoder;
  // 2 motors, 1 rollers, 1 to move extendable intake out
  // rollers need to be able to switch directions

  public IntakeSubsystem() {
    m_rollerMotor = new CANSparkMax(Constants.Intake.kRollerMotorID, MotorType.kBrushless);
    m_intakeMotor = new CANSparkMax(Constants.Intake.kIntakeMotorID, MotorType.kBrushless);
    m_intakeMotor.restoreFactoryDefaults();
    m_intakeMotor.setIdleMode(IdleMode.kBrake); 
    m_intakeMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_intakeMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,
                       (float)Constants.Intake.kUpperLimit); 
    m_intakeMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_intakeMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
                       (float)Constants.Intake.kBottomLimit);
    m_intakeMotor.setSmartCurrentLimit(60);
    m_intakeMotor.setSecondaryCurrentLimit(60, 0);

    m_intakeMotorEncoder = m_intakeMotor.getEncoder();
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  public Command runRollersCommand(Double speed) {
    return Commands.run(
      () -> {
        m_rollerMotor.set(speed);
      },
      this);
  }
  public Command moveIntakeCommand(Double speed) {
    return Commands.run(
      () -> {
        m_intakeMotor.set(speed);
      },
      this);
  }

  public Double getEncoderPosition(){
    return m_intakeMotorEncoder.getPosition();
  }

  public void resetEncoder() {
    m_intakeMotorEncoder.setPosition(0);
  }

  public void enableSoftLimitsCommand(boolean enable){
    if (enable) {
      m_intakeMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
      m_intakeMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    } else {
      m_intakeMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
      m_intakeMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
    }
  }

  public Command moveIntakeOverrideCommand() {
    return Commands.runOnce(
      () -> {
        enableSoftLimitsCommand(false);
      }
    )
    .andThen(
      moveIntakeCommand(-0.15)
    )
    .finallyDo(
      () -> {
        enableSoftLimitsCommand(true);
        resetEncoder();
        moveIntakeCommand(0.0);
      }
    )
    .withName("tiltLauncherOverride");
  }

  public Command moveIntakeOut(Double speed) {
    return moveIntakeCommand(speed)
      .until(() -> getEncoderPosition() >= Constants.Intake.kUpperLimit)
      .andThen(moveIntakeCommand(0.0));
  }

  public Command moveIntakeIn(Double speed) {
    return moveIntakeCommand(speed)
      .until(() -> getEncoderPosition() <= Constants.Intake.kBottomLimit)
      .andThen(moveIntakeCommand(0.0));
  }

  private void updateTelemetry() {
    // TODO: send subsystem telemetry data to the dashboard as needed
    // ex: SmartDashboard.putString("Robot/Example/String", "TEST");
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // TODO: send subsystem data to be logged on the robot as needed
    // ex: builder.addDoubleProperty("Double", this::getSomeDoubleValue, null);
  }
}
