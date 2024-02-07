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

public class LauncherSubsystem extends SubsystemBase {
  private final CANSparkMax m_armMotor;
  private final RelativeEncoder m_armEncoder;
  private final SparkPIDController m_armPIDController;
  private final CANSparkMax m_topRollerMotor;
  private final CANSparkMax m_bottomRollerMotor;

  private boolean m_isAlignedToTarget = false;

  public LauncherSubsystem() {
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

    m_topRollerMotor = new CANSparkMax(Constants.Launcher.kTopRollerMotorCANId, MotorType.kBrushless);
    m_topRollerMotor.restoreFactoryDefaults();
    m_topRollerMotor.setIdleMode(Constants.Launcher.kTopRollerMotorIdleMode); 
    m_topRollerMotor.setSmartCurrentLimit(Constants.Launcher.kTopRollerMotorCurrentLimit);
    m_topRollerMotor.setSecondaryCurrentLimit(Constants.Launcher.kTopRollerMotorCurrentLimit);

    m_bottomRollerMotor = new CANSparkMax(Constants.Launcher.kBottomRollerMotorCANId, MotorType.kBrushless);
    m_bottomRollerMotor.restoreFactoryDefaults();
    m_bottomRollerMotor.setIdleMode(Constants.Launcher.kBottomRollerMotorIdleMode); 
    m_bottomRollerMotor.setSmartCurrentLimit(Constants.Launcher.kBottomRollerMotorCurrentLimit);
    m_bottomRollerMotor.setSecondaryCurrentLimit(Constants.Launcher.kBottomRollerMotorCurrentLimit);
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  public Command alignToTargetCommand(Supplier<Pose2d> currentPose, Pose3d targetPose) {
    return
      runOnce(() -> {
        m_isAlignedToTarget = false;
      })
      .andThen(
        this
          .run(() -> {
            double position = calculateArmPosition(currentPose.get(), targetPose);
            m_armPIDController.setReference(position, ControlType.kSmartMotion);
            m_isAlignedToTarget = Math.abs(m_armEncoder.getPosition() - position) < 0.1; // TODO: determine if this is correct tolerance
          })
          .until(() -> m_isAlignedToTarget)
          .finallyDo(() -> m_armMotor.set(0.0))
      )
      .withName("AlignLauncherElevationToTarget");
  }

  public double calculateArmPosition(Pose2d currentPose, Pose3d targetPose) {
    // TODO: adjust current pose with transform defined in constants (kLauncherToRobotTransform3d) ... launcher is higher in the robot than the ground
    // TODO: use both poses in 3D to calculate height differential for "rise"
    // TOOD: use both poses in 2D to calculate distance differential for "run"
    // TODO: use atan2(rise/run) to get elevation angle
    // TODO: confirm angle adjustment factor needed
    // TODO: convert angle into arm position
    return 0.0;
  }

  public Command runRollersCommand(double topRollerSpeed, double bottomRollerSpeed) {
    return 
      run(() -> {
        m_topRollerMotor.set(topRollerSpeed * Constants.Launcher.kTopRollerMotorMaxOutput);
        m_bottomRollerMotor.set(bottomRollerSpeed * Constants.Launcher.kBottomRollerMotorMaxOutput);
      })
      .withName("RunLauncherRollers");
  }

  public Command resetCommand() {
    return 
      startEnd(
        () -> {
          m_armMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
          m_armMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
          m_armMotor.set(-0.1);
        }, 
        () -> {
          m_armEncoder.setPosition(0);
          m_armMotor.set(0.0);
          m_armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
          m_armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        }
      )
      .withName("ResetLauncher");
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
