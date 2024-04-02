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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.common.Utils;

public class LauncherArmSubsystem extends SubsystemBase {
  private final CANSparkMax m_armMotor;
  private final RelativeEncoder m_armEncoder;
  private final SparkPIDController m_armPIDController;

  private double[] m_distances;
  private double[] m_positions;
  private boolean m_isAlignedToTarget = false;
  private boolean m_hasInitialZeroReset = false;

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
    m_armPIDController.setP(Constants.Launcher.kArmMotorPIDConstants.P());
    m_armPIDController.setD(Constants.Launcher.kArmMotorPIDConstants.D());
    m_armPIDController.setOutputRange(Constants.Launcher.kArmMotorMaxReverseOutput, Constants.Launcher.kArmMotorMaxForwardOutput);
    m_armPIDController.setSmartMotionMaxVelocity(Constants.Launcher.kArmMotorSmartMotionMaxVelocity, 0);
    m_armPIDController.setSmartMotionMaxAccel(Constants.Launcher.kArmMotorSmartMotionMaxAccel, 0);
    m_armMotor.burnFlash();

    m_distances = new double[Constants.Launcher.kArmPositions.length];
    m_positions = new double[Constants.Launcher.kArmPositions.length];
    for (int i = 0; i < Constants.Launcher.kArmPositions.length; i++) {
      m_distances[i] = Constants.Launcher.kArmPositions[i].distance();
      m_positions[i] = Constants.Launcher.kArmPositions[i].position();
    }   

    SmartDashboard.putString("Robot/Launcher/Arm/Positions", Utils.objectToJson(Constants.Launcher.kArmPositions));
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  public Command alignManualCommand(Supplier<Double> speed) {
    return 
    run(() -> {
      m_armMotor.set(speed.get() * 0.5);
    })
    .beforeStarting(() -> m_isAlignedToTarget = false)
    .finallyDo(() -> { reset(); })
    .withName("AlignLauncherArmManual");
  }

  public Command alignToPositionCommand(double position) {
    return 
    run(() -> { 
      m_armPIDController.setReference(position, ControlType.kSmartMotion); 
      m_isAlignedToTarget = Math.abs(m_armEncoder.getPosition() - position) <= Constants.Launcher.kArmTargetAlignmentPositionTolerance;
    })
    .beforeStarting(() -> m_isAlignedToTarget = false)
    .finallyDo(() -> { reset(); })
    .withName("AlignLauncherArmToPosition");
  }

  public Command alignToPositionAutoCommand(double position) {
    return 
    alignToPositionCommand(position)
    .until(() -> m_isAlignedToTarget);
  }
  
  public Command alignToSpeakerCommand(Supplier<Double> targetDistance) {
    return
    run(() -> {
      double position = calculatePositionForSpeaker(targetDistance.get()); 
      m_armPIDController.setReference(position, ControlType.kSmartMotion);
      m_isAlignedToTarget = Math.abs(m_armEncoder.getPosition() - position) < Constants.Launcher.kArmTargetAlignmentPositionTolerance;
    })
    .beforeStarting(() -> m_isAlignedToTarget = false)
    .finallyDo(() -> { reset(); })
    .withName("AlignLauncherArmToTarget");
  }

  public Command alignToSpeakerAutoCommand(Supplier<Double> targetDistance) {
    return
    alignToSpeakerCommand(targetDistance)
    .until(() -> m_isAlignedToTarget);
  }

  private double calculatePositionForSpeaker(double distance) {
    double position = Utils.getLinearInterpolation(m_distances, m_positions, distance);
    return 
    Utils.isValueInRange(position, Constants.Launcher.kArmMotorReverseSoftLimit, Constants.Launcher.kArmMotorForwardSoftLimit) 
      ? position 
      : Constants.Launcher.kArmPositionSubwoofer;
  }

  public double getArmPosition() {
    return m_armEncoder.getPosition();
  }

  public boolean isAlignedToTarget() {
    return m_isAlignedToTarget;
  }

  public void clearTargetAlignment() {
    m_isAlignedToTarget = false;
  }

  public boolean hasInitialZeroReset() {
    return m_hasInitialZeroReset;
  }

  public Command resetZeroCommand() {
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
        m_hasInitialZeroReset = true;
      }
    )
    .withName("ResetLauncherArm");
  }

  public void reset() {
    m_armMotor.set(0.0);
  }

  private void updateTelemetry() {
    SmartDashboard.putNumber("Robot/Launcher/Arm/Position", m_armEncoder.getPosition());
    SmartDashboard.putBoolean("Robot/Launcher/Arm/IsAlignedToTarget", m_isAlignedToTarget);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
