package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  
  // TODO: add position safety check after robot power on to not allow operation unless soft limit reset to zero has been confirmed (manual or auto)

  public ClimberSubsystem() {
    m_armMotor = new CANSparkMax(Constants.Arm.kArmMotorCANId, MotorType.kBrushless);
    m_armMotor.restoreFactoryDefaults();
    m_armMotor.setIdleMode(Constants.Arm.kArmMotorIdleMode); 
    m_armMotor.setSmartCurrentLimit(Constants.Arm.kArmMotorCurrentLimit);
    m_armMotor.setSecondaryCurrentLimit(Constants.Arm.kArmMotorCurrentLimit);
    // m_armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    // m_armMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.Arm.kArmMotorForwardSoftLimit); 
    // m_armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    // m_armMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.Arm.kArmMotorReverseSoftLimit);

    m_armEncoder = m_armMotor.getEncoder();
    m_armEncoder.setPositionConversionFactor(Constants.Arm.kArmMotorPositionConversionFactor);
    m_armEncoder.setVelocityConversionFactor(Constants.Arm.kArmMotorVelocityConversionFactor);
    
    m_armPIDController = m_armMotor.getPIDController();
    m_armPIDController.setFeedbackDevice(m_armEncoder);
    m_armPIDController.setP(Constants.Arm.kArmMotorPIDConstants.P);
    m_armPIDController.setD(Constants.Arm.kArmMotorPIDConstants.D);
    m_armPIDController.setOutputRange(Constants.Arm.kArmMotorMinOutput, Constants.Arm.kArmMotorMaxOutput);
    m_armPIDController.setSmartMotionMaxVelocity(Constants.Arm.kArmMotorSmartMotionMaxVelocity, 0);
    m_armPIDController.setSmartMotionMaxAccel(Constants.Arm.kArmMotorSmartMotionMaxAccel, 0);

    // TODO: get correct motor configuration for brushed/bag motor
    m_rollerMotor = new CANSparkMax(Constants.Arm.kRollerMotorCANId, MotorType.kBrushless);
    // m_rollerMotor.restoreFactoryDefaults();
    // m_rollerMotor.setIdleMode(Constants.Arm.kRollerMotorIdleMode); 
    // m_rollerMotor.setSmartCurrentLimit(Constants.Arm.kRollerMotorCurrentLimit);
    // m_rollerMotor.setSecondaryCurrentLimit(Constants.Arm.kRollerMotorCurrentLimit);
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  public Command moveArmManualCommand(Supplier<Double> speed) {
    return 
    run(() -> {
      m_armMotor.set(speed.get() / 2); // TODO: test/tune speed ratio to determine why cutting in half is needed
    })
    .finallyDo(() -> m_armMotor.set(0.0))
    .withName("MoveClimberArmManual");
  }

  public Command moveArmToPositionCommand(double position) {
    return
    run(() -> {
      m_armPIDController.setReference(position, ControlType.kSmartMotion);
      m_isArmAlignedToPosition = Math.abs(m_armEncoder.getPosition() - position) < 0.1; // TODO: determine if this is correct tolerance
    })
    .beforeStarting(() -> m_isArmAlignedToPosition = false)
    .until(() -> m_isArmAlignedToPosition)
    .finallyDo(() -> m_armMotor.set(0.0))
    .withName("MoveClimberArmToPosition");
  }

  public Command runRollersCommand(MotorDirection direction) {
    return 
    startEnd(() -> {
      // TODO: determine correct direction of travel for inward/outward with motor
      m_rollerMotor.set(
        direction == MotorDirection.Forward ? 
        Constants.Arm.kRollerMotorMaxOutput : 
        Constants.Arm.kRollerMotorMinOutput
      ); 
    }, () -> {
      m_rollerMotor.set(0.0);
    })
    .until(() -> false) // TODO: determine current limit on bag motor to stop rollers when performing inward intake
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
    })
    .withName("ResetClimberArm");
  }

  private void updateTelemetry() {
    SmartDashboard.putNumber("Robot/Climber/Arm/Position", m_armEncoder.getPosition());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
