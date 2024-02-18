package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.common.Utils;

public class LauncherArmSubsystem extends SubsystemBase {
  private final CANSparkMax m_armMotor;
  private final RelativeEncoder m_armEncoder;
  private final SparkPIDController m_armPIDController;

  private boolean m_isAlignedToTarget = false;

  // TODO: add position safety check after robot power on to not allow operation unless soft limit reset to zero has been confirmed (manual or auto)

  public LauncherArmSubsystem() {
    m_armMotor = new CANSparkMax(Constants.Launcher.kArmMotorCANId, MotorType.kBrushless);
    m_armMotor.restoreFactoryDefaults();
    m_armMotor.setIdleMode(Constants.Launcher.kArmMotorIdleMode); 
    m_armMotor.setSmartCurrentLimit(Constants.Launcher.kArmMotorCurrentLimit);
    m_armMotor.setSecondaryCurrentLimit(Constants.Launcher.kArmMotorCurrentLimit);
    m_armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_armMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.Launcher.kArmMotorForwardSoftLimit); 
    m_armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_armMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.Launcher.kArmMotorReverseSoftLimit);

    m_armEncoder = m_armMotor.getEncoder();
    m_armEncoder.setPositionConversionFactor(Constants.Launcher.kArmMotorPositionConversionFactor);
    m_armEncoder.setVelocityConversionFactor(Constants.Launcher.kArmMotorVelocityConversionFactor);
    
    m_armPIDController = m_armMotor.getPIDController();
    m_armPIDController.setFeedbackDevice(m_armEncoder);
    m_armPIDController.setP(Constants.Launcher.kArmMotorPIDConstants.P);
    m_armPIDController.setD(Constants.Launcher.kArmMotorPIDConstants.D);
    m_armPIDController.setOutputRange(Constants.Launcher.kArmMotorMinOutput, Constants.Launcher.kArmMotorMaxOutput);
    m_armPIDController.setSmartMotionMaxVelocity(Constants.Launcher.kArmMotorSmartMotionMaxVelocity, 0);
    m_armPIDController.setSmartMotionMaxAccel(Constants.Launcher.kArmMotorSmartMotionMaxAccel, 0);
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  public Command alignManualCommand(Supplier<Double> speed) {
    return 
    run(() -> {
      m_armMotor.set(speed.get() / 2);
    })
    .finallyDo(() -> m_armMotor.set(0.0))
    .withName("AlignLauncherArmManual");
  }

  public Command alignToPositionCommand(Double position) {
    return 
    run(() -> m_armPIDController.setReference(position, ControlType.kSmartMotion))
    .withName("AlignLauncherArmToPosition");
  }

  public Command alignToTargetCommand(Supplier<Pose2d> robotPose, Pose3d targetPose) {
    return
    run(() -> {
      double position = calculateArmPosition(robotPose.get(), targetPose); // TODO: determine through testing if this should be first-time-only calculation
      m_armPIDController.setReference(position, ControlType.kSmartMotion);
      m_isAlignedToTarget = Math.abs(m_armEncoder.getPosition() - position) < 0.1; // TODO: determine if this is correct tolerance
    })
    .beforeStarting(() -> m_isAlignedToTarget = false)
    .until(() -> m_isAlignedToTarget)
    //.unless(() -> !m_isTiltLocked)
    .finallyDo(() -> m_armMotor.set(0.0))
    .withName("AlignLauncherArmToTarget");

    
  }

  private double calculateArmPosition(Pose2d robotPose, Pose3d targetPose) {
    double pitch = Math.toDegrees(Utils.getTargetRotation(robotPose, targetPose).getY());
    double position = pitch * Constants.Launcher.kArmPositionFromTargetPitchConversionFactor;
    return 
    Utils.isValueBetween(position, Constants.Launcher.kArmMotorReverseSoftLimit, Constants.Launcher.kArmMotorForwardSoftLimit) 
      ? position 
      : Constants.Launcher.kArmPositionIntake;
  }

  public Command resetCommand() {
    return 
    startEnd(
      () -> {
        Utils.enableSoftLimits(m_armMotor, false);
        m_armMotor.set(-0.1);
      }, 
      () -> {
        m_armEncoder.setPosition(0);
        m_armMotor.set(0.0);
        Utils.enableSoftLimits(m_armMotor, true);
      }
    )
    .withName("ResetLauncherArm");
  }

  private void updateTelemetry() {
    SmartDashboard.putNumber("Robot/Launcher/Arm/Position", m_armEncoder.getPosition());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
