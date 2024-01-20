// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax m_leadScrewMotor;
  private final CANSparkMax m_rollerMotor;
  private final SparkPIDController m_leadScrewPID;
  private final RelativeEncoder m_leadScrewMotorEncoder;

  private double m_velocity = (33.0 / Constants.Arm.kRotationsToInches) * 60;
  private double m_acceleration = (100.0 / Constants.Arm.kVelocityConversion);
  
  public ArmSubsystem() {
    m_leadScrewMotor = new CANSparkMax(Constants.Arm.kLeadScrewMotorID, MotorType.kBrushless);
    m_leadScrewMotor.restoreFactoryDefaults();
    m_leadScrewMotor.setIdleMode(IdleMode.kBrake); 
    m_leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_leadScrewMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,
                       (float)Constants.Arm.kLeadScrewUpperLimit); 
    m_leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_leadScrewMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
                       (float)Constants.Arm.kLeadScrewLowerLimit);
    m_leadScrewMotor.setSmartCurrentLimit(60);
    m_leadScrewMotor.setSecondaryCurrentLimit(60, 0);

    m_rollerMotor = new CANSparkMax(Constants.Arm.kRollerMotorID, MotorType.kBrushed);

    m_leadScrewMotorEncoder = m_leadScrewMotor.getEncoder();
    m_leadScrewMotorEncoder.setPositionConversionFactor(Constants.Arm.kRotationsToInches);
    m_leadScrewMotorEncoder.setVelocityConversionFactor(Constants.Arm.kVelocityConversion);
    
    m_leadScrewPID = m_leadScrewMotor.getPIDController();
    m_leadScrewPID.setSmartMotionMaxAccel(m_acceleration, 0);
    m_leadScrewPID.setFeedbackDevice(m_leadScrewMotorEncoder);
    m_leadScrewPID.setP(Constants.Arm.kLeadScrewP);
    m_leadScrewPID.setD(Constants.Arm.kLeadScrewD);
    m_leadScrewPID.setOutputRange(Constants.Arm.kLeadScrewMinOutput,
                                  Constants.Arm.kLeadScrewMaxOutput);
  }

    @Override
  public void periodic() {
    updateTelemetry();
  }

  public Command runLeadScrewCommand(double speed) { 
    return Commands.run(
      () -> m_leadScrewMotor.set(speed), 
      this)
      .withName("runLeadScrew: Double"); // TODO: Change name??
  }

  public Command runLeadScrewCommand(DoubleSupplier speeds) { 
    return Commands.run(
      () -> m_leadScrewMotor.set(speeds.getAsDouble()), // TODO: Make negative?
      this)
      .withName("runLeadScrew: DoubleSupplier"); // TODO: Change name??
  }

  public void setDesiredPosition(double position, double speed) {
    speed *= m_velocity;
    m_leadScrewPID.setSmartMotionMaxVelocity(speed, 0);
    m_leadScrewPID.setReference(position, CANSparkMax.ControlType.kSmartMotion);
  }

  public void setDesiredPosition(double position) {
    m_leadScrewPID.setSmartMotionMaxVelocity(m_velocity, 0);
    m_leadScrewPID.setReference(position, CANSparkMax.ControlType.kSmartMotion);
  }

  // In inches
  public double getEncoderPosition() {
    return m_leadScrewMotorEncoder.getPosition();
  }

  public void resetEncoder() {
    m_leadScrewMotorEncoder.setPosition(0);
  }

  public void enableSoftLimitsCommand(boolean enable){
    if (enable) {
      m_leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
      m_leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    } else {
      m_leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
      m_leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
    }
  }

  public Command moveArmOverrideCommand() {
    return Commands.runOnce(
      () -> {
        enableSoftLimitsCommand(false);
      }
    )
    .andThen(
          runLeadScrewCommand(-0.15)
    )
    .finallyDo(
      () -> {
        enableSoftLimitsCommand(true);
        resetEncoder();
        runLeadScrewCommand(0.0);
      }
    )
    .withName("moveArmOverride");
  }

  public Command moveArmToHeightCommand(double speed, double position) {
    return Commands.run(
      () -> {
        setDesiredPosition(position, speed);
      }
    )
    .until(() -> (Math.abs(getEncoderPosition() - position) < 0.1))
    .andThen(() -> runLeadScrewCommand(0.0))
    .withName("moveArmToHeight");
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
