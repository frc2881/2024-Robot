package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
  private final CANSparkMax m_rollerMotor;
  private final CANSparkMax m_armMotor;
  private final RelativeEncoder m_armEncoder;
  private final SparkPIDController m_armPIDController;

  public FeederSubsystem() {
    m_rollerMotor = new CANSparkMax(Constants.Feeder.kRollerCanId, MotorType.kBrushless);
    m_rollerMotor.restoreFactoryDefaults();
    m_rollerMotor.setSmartCurrentLimit(60);
    m_rollerMotor.setSecondaryCurrentLimit(60, 0);

    m_armMotor = new CANSparkMax(Constants.Feeder.kArmCanId, MotorType.kBrushless);
    m_armMotor.restoreFactoryDefaults();
    m_armMotor.setIdleMode(IdleMode.kBrake); 
    m_armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)Constants.Feeder.kArmForwardLimit); 
    m_armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float)Constants.Feeder.kArmReverseLimit);
    m_armMotor.setSmartCurrentLimit(60);
    m_armMotor.setSecondaryCurrentLimit(60, 0);

    m_armPIDController = m_armMotor.getPIDController();
    m_armPIDController.setP(0.1);
    m_armPIDController.setOutputRange(Constants.Feeder.kArmMinOutput, Constants.Feeder.kArmMaxOutput);

    m_armEncoder = m_armMotor.getEncoder();
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  public Command startFeederCommand() {
    return Commands.run(() -> {
      m_armPIDController.setReference(Constants.Feeder.kArmForwardLimit, CANSparkBase.ControlType.kPosition);
      m_rollerMotor.set(Constants.Feeder.kRollerMaxOutput);
    }, this)
    .withName("StartFeeder");
  }

  public Command stopFeederCommand() {
    return Commands.run(() -> {
      m_armPIDController.setReference(Constants.Feeder.kArmReverseLimit, CANSparkBase.ControlType.kPosition);
      m_rollerMotor.set(0.0);
    }, this)
    .withTimeout(2)
    .andThen(() -> m_armMotor.set(0.0))
    .withName("StopFeeder");
  }

  public Command resetFeederCommand() {
    return Commands.runOnce(
      () -> setSoftLimits(false)
    )
    .andThen(
      Commands.run(() -> m_armMotor.set(-0.1), this)
    )
    .finallyDo(
      () -> {
        setSoftLimits(true);
        resetEncoder();
        m_armMotor.set(-0.0);
      }
    )
    .withName("ResetFeeder");
  }

  private void resetEncoder() {
    m_armEncoder.setPosition(0);
  }

  private void setSoftLimits(boolean isEnabled){
    m_armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
    m_armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
  }

  private void updateTelemetry() {
    SmartDashboard.putNumber("Robot/Feeder/Arm/Position", m_armEncoder.getPosition());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // TODO: send subsystem data to be logged on the robot as needed
    // ex: builder.addDoubleProperty("Double", this::getSomeDoubleValue, null);
  }
}
