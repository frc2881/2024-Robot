// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax m_rollerMotor;
  private final CANSparkMax m_intakeMotor;
  // 2 motors, 1 rollers, 1 to move extendable intake out
  // rollers need to be able to switch directions

  public IntakeSubsystem() {
    m_rollerMotor = new CANSparkMax(Constants.Launcher.kLeadScrewMotorID, MotorType.kBrushless);
    m_intakeMotor = new CANSparkMax(Constants.Launcher.kLeadScrewMotorID, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  public Command runRollers(double speed) {
    return Commands.run(
      () -> {
        m_rollerMotor.set(speed);
      },
      this);
  }

  public void reset() {
    // TODO: reset the subsystem if needed
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
