package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.common.Enums.MotorDirection;
import frc.robot.lib.common.Utils;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax m_armMotor;
  private final SparkPIDController m_armPIDController;
  private final RelativeEncoder m_armEncoder;
  private final CANSparkMax m_rollerMotor;

  private boolean m_isArmAlignedToPosition = false;
  private boolean m_hasInitialReset = false;
  
  public ClimberSubsystem() {
    m_armMotor = new CANSparkMax(Constants.Climber.kArmMotorCANId, MotorType.kBrushless);
    m_armMotor.restoreFactoryDefaults();
    m_armMotor.setIdleMode(Constants.Climber.kArmMotorIdleMode); 
    m_armMotor.setSmartCurrentLimit(Constants.Climber.kArmMotorCurrentLimit);
    m_armMotor.setSecondaryCurrentLimit(Constants.Climber.kArmMotorCurrentLimit);
    m_armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_armMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.Climber.kArmMotorForwardSoftLimit); 
    m_armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_armMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.Climber.kArmMotorReverseSoftLimit);
    m_armEncoder = m_armMotor.getEncoder();
    m_armEncoder.setPositionConversionFactor(Constants.Climber.kArmMotorPositionConversionFactor);
    m_armEncoder.setVelocityConversionFactor(Constants.Climber.kArmMotorVelocityConversionFactor);
    m_armPIDController = m_armMotor.getPIDController();
    m_armPIDController.setFeedbackDevice(m_armEncoder);
    m_armPIDController.setP(Constants.Climber.kArmMotorPIDConstants.P());
    m_armPIDController.setD(Constants.Climber.kArmMotorPIDConstants.D());
    m_armPIDController.setOutputRange(Constants.Climber.kArmMotorMinOutput, Constants.Climber.kArmMotorMaxOutput);
    m_armPIDController.setSmartMotionMaxVelocity(Constants.Climber.kArmMotorSmartMotionMaxVelocity, 0);
    m_armPIDController.setSmartMotionMaxAccel(Constants.Climber.kArmMotorSmartMotionMaxAccel, 0);

    m_armMotor.burnFlash();

    m_rollerMotor = new CANSparkMax(Constants.Climber.kRollerMotorCANId, MotorType.kBrushed);
    // m_rollerMotor.restoreFactoryDefaults();
    // m_rollerMotor.setIdleMode(Constants.Arm.kRollerMotorIdleMode); 
    // m_rollerMotor.setSmartCurrentLimit(Constants.Arm.kRollerMotorCurrentLimit);
    // m_rollerMotor.setSecondaryCurrentLimit(Constants.Arm.kRollerMotorCurrentLimit);
    // m_rollerMotor.burnFlash();
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  public Command moveArmManualCommand(Supplier<Double> speed) {
    return 
    run(() -> {
      m_armMotor.set(speed.get() * 0.5);
    })
    .finallyDo(() -> m_armMotor.set(0.0))
    .withName("MoveClimberArmManual");
  }

  public Command moveArmToPositionCommand(double position) {
    return
    run(() -> {
      m_armPIDController.setReference(position, ControlType.kSmartMotion);
      m_isArmAlignedToPosition = Math.abs(m_armEncoder.getPosition() - position) < 0.1;
    })
    .beforeStarting(() -> m_isArmAlignedToPosition = false)
    .until(() -> m_isArmAlignedToPosition)
    .finallyDo(() -> m_armMotor.set(0.0))
    .withName("MoveClimberArmToPosition");
  }

  public Command runRollersCommand(MotorDirection direction) {
    return 
    Commands.startEnd(() -> {
      m_rollerMotor.set(
        direction == MotorDirection.Forward ? 
        Constants.Climber.kRollerMotorMaxOutput : 
        Constants.Climber.kRollerMotorMinOutput
      ); 
    }, () -> {
      m_rollerMotor.set(0.0);
    })
    .withName("RunClimberArmRollers");
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
      m_hasInitialReset = true;
    })
    .withName("ResetClimberArm");
  }

  public boolean hasInitialReset() {
    return m_hasInitialReset;
  }

  public void reset() {
    m_armMotor.set(0.0);
    m_rollerMotor.set(0.0);
  }

  private void updateTelemetry() {
    SmartDashboard.putNumber("Robot/Climber/Arm/Position", m_armEncoder.getPosition());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
