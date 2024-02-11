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
  private boolean m_targetIsSpeaker = true;

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

  public Command tiltLauncherCommand(Supplier<Double> speed) {
    return 
    run(
      () -> {
        m_armMotor.set(speed.get() / 2); // TODO: test/tune speed ratio to determine why cutting in half is needed
      })
      .finallyDo(() -> m_armMotor.set(0.0))
      .withName("TiltLauncher");
  }

  public Command alignToDefaultPositionCommand() {
    return 
    runOnce(
      () -> m_armPIDController.setReference(Constants.Launcher.kDefaultPosition, ControlType.kSmartMotion)
    )
    .withName("AlignLaunchedToDefaultPosition");
  }

  public Command alignToSpeakerPositionCommand() {
    return 
    run(
      () -> m_armPIDController.setReference(12.75, ControlType.kSmartMotion)
    )
    .withName("AlignLaunchedToSpeakerPosition");
  }

  public Command alignToTargetCommand(Supplier<Pose2d> currentPose, Pose3d targetPose) {
    return
    run(() -> {
      double position = calculateArmPosition(currentPose.get(), targetPose); // TODO: determine with testing whether this needs to be a first-time calculation only or ok to do on-the-fly
      m_armPIDController.setReference(position, ControlType.kSmartMotion);
      m_isAlignedToTarget = Math.abs(m_armEncoder.getPosition() - position) < 0.1; // TODO: determine if this is correct tolerance
    })
    .beforeStarting(() -> m_isAlignedToTarget = false)
    .until(() -> m_isAlignedToTarget)
    .finallyDo(() -> m_armMotor.set(0.0))
    .withName("AlignLauncherElevationToTarget");
  }

  private double calculateArmPosition(Pose2d currentPose, Pose3d targetPose) {
    // TODO: if needed, adjust current pose with transform defined in constants (kLauncherToRobotTransform3d) ... launcher is higher in the robot than the ground
    double height = targetPose.minus(new Pose3d(currentPose)).getZ();
    double distance = currentPose.getTranslation().getDistance(targetPose.toPose2d().getTranslation()); 
    double angle = Math.toDegrees(Math.atan2(height, distance)); 
    // TODO: confirm angle adjustment factor needed based on field testing - may need a "fudge factor" based on mechanical and aerodynamics
    // TODO: convert the adjusted angle into arm reference position to set for the lead screw
    return 0.0;
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
    .withName("ResetLauncher");
  }

  public void switchTarget() {
    m_targetIsSpeaker = !m_targetIsSpeaker;
  }

  public boolean isTargetSpeaker() {
    return m_targetIsSpeaker;
  }

  private void updateTelemetry() {
    SmartDashboard.putNumber("Robot/Launcher/Arm/Position", m_armEncoder.getPosition());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // TODO: send subsystem data to be logged on the robot as needed
    // ex: builder.addDoubleProperty("Double", this::getSomeDoubleValue, null);
  }
}
