package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.lib.common.Utils;

public class FeederSubsystem extends SubsystemBase {
  private final CANSparkMax m_armMotor;
  private final RelativeEncoder m_armEncoder;
  private final SparkPIDController m_armPIDController;
  private final CANSparkMax m_rollerMotor;

  public FeederSubsystem() {
    m_armMotor = new CANSparkMax(Constants.Feeder.kArmMotorCANId, MotorType.kBrushless);
    m_armMotor.restoreFactoryDefaults();
    m_armMotor.setIdleMode(Constants.Feeder.kArmMotorIdleMode); 
    m_armMotor.setSmartCurrentLimit(Constants.Feeder.kArmMotorCurrentLimit);
    m_armMotor.setSecondaryCurrentLimit(Constants.Feeder.kArmMotorCurrentLimit);
    m_armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_armMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.Feeder.kArmMotorForwardSoftLimit); 
    m_armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_armMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.Feeder.kArmMotorReverseSoftLimit);
    
    m_armPIDController = m_armMotor.getPIDController();
    m_armPIDController.setP(Constants.Feeder.kArmMotorPIDConstants.P);
    m_armPIDController.setI(Constants.Feeder.kArmMotorPIDConstants.I);
    m_armPIDController.setD(Constants.Feeder.kArmMotorPIDConstants.D);
    m_armPIDController.setOutputRange(Constants.Feeder.kArmMotorMinOutput, Constants.Feeder.kArmMotorMaxOutput);

    m_armEncoder = m_armMotor.getEncoder();

    m_rollerMotor = new CANSparkMax(Constants.Feeder.kRollerMotorCANId, MotorType.kBrushless);
    m_rollerMotor.restoreFactoryDefaults();
    m_rollerMotor.setIdleMode(Constants.Feeder.kArmMotorIdleMode); 
    m_rollerMotor.setSmartCurrentLimit(Constants.Feeder.kRollerMotorCurrentLimit);
    m_rollerMotor.setSecondaryCurrentLimit(Constants.Feeder.kRollerMotorCurrentLimit);
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  public Command runCommand() {
    return
    run(() -> {
      m_armPIDController.setReference(Constants.Feeder.kArmMotorForwardSoftLimit, ControlType.kPosition);
      m_rollerMotor.set(Constants.Feeder.kRollerMotorMaxOutput);
    })
    .withName("StartFeeder");
  }

  public Command stopCommand() {
    return
    run(() -> {
      m_armPIDController.setReference(Constants.Feeder.kArmMotorReverseSoftLimit, ControlType.kPosition);
      m_rollerMotor.set(0.0);
    })
    .withTimeout(3.0)
    .finallyDo(() -> m_armMotor.set(0.0))
    .withName("StopFeeder");
  }

  public Command moveArmOutCommand() {
    return 
    run(() -> {
      m_armPIDController.setReference(Constants.Feeder.kArmMotorForwardSoftLimit, ControlType.kPosition);
    })
    .withName("MoveFeederOut");
  }

  public Command moveArmInCommand() {
    return 
    run(() -> {
      m_armPIDController.setReference(Constants.Feeder.kArmMotorReverseSoftLimit, ControlType.kPosition);
    })
    .withName("MoveFeederIn");
  }

  public Command resetCommand() {
    return
    startEnd(() -> {
      Utils.enableSoftLimits(m_armMotor, false);
      m_armMotor.set(-0.1);
    }, () -> {
      m_armEncoder.setPosition(0);
      m_armMotor.set(0.0);
      Utils.enableSoftLimits(m_armMotor, true);
    })
    .withName("ResetFeeder");
  }

  public void reset() {
    m_armMotor.set(0.0);
    m_rollerMotor.set(0.0);
  }

  private void updateTelemetry() {
    SmartDashboard.putNumber("Robot/Feeder/Arm/Position", m_armEncoder.getPosition());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
