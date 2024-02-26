package frc.robot.lib.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.common.Utils;
import frc.robot.lib.common.Enums.SwerveModuleLocation;
import frc.robot.Constants;

public class SwerveModule implements Sendable {
  private static List<CANSparkBase> m_motorControllers = new ArrayList<CANSparkBase>();

  private final SwerveModuleLocation m_location;

  private final CANSparkFlex m_drivingMotor;
  private final CANSparkMax m_turningMotor;
  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;
  private final SparkPIDController m_drivingPIDController;
  private final SparkPIDController m_turningPIDController;

  private double m_turningOffset = 0;
  private double m_setSpeed = 0;

  public SwerveModule(SwerveModuleLocation location, int drivingMotorCanId, int turningMotorCanId, double turningOffset) {
    m_location = location;

    String logPrefix = "SwerveModule:" + m_location.toString();

    m_drivingMotor = new CANSparkFlex(drivingMotorCanId, MotorType.kBrushless);
    m_motorControllers.add(m_drivingMotor);
    Utils.setConfiguration(m_drivingMotor.restoreFactoryDefaults(), logPrefix + ":m_drivingSparkFlex.restoreFactoryDefaults");
    m_drivingEncoder = m_drivingMotor.getEncoder();
    Utils.setConfiguration(m_drivingEncoder.setPositionConversionFactor(Constants.Drive.SwerveModule.kDrivingEncoderPositionConversionFactor), logPrefix + ":m_drivingEncoder.setPositionConversionFactor");
    Utils.setConfiguration(m_drivingEncoder.setVelocityConversionFactor(Constants.Drive.SwerveModule.kDrivingEncoderVelocityConversionFactor), logPrefix + ":m_drivingEncoder.setVelocityConversionFactor");
    Utils.setConfiguration(m_drivingEncoder.setMeasurementPeriod(16), logPrefix + ":m_drivingEncoder.setMeasurementPeriod");
    Utils.setConfiguration(m_drivingEncoder.setAverageDepth(2), logPrefix + ":m_drivingEncoder.setAverageDepth");
    m_drivingPIDController = m_drivingMotor.getPIDController();
    Utils.setConfiguration(m_drivingPIDController.setOutputRange(Constants.Drive.SwerveModule.kDrivingMotorMinOutput, Constants.Drive.SwerveModule.kDrivingMotorMaxOutput), logPrefix + ":m_drivingPIDController.setOutputRange");
    Utils.setConfiguration(m_drivingPIDController.setP(Constants.Drive.SwerveModule.kDrivingMotorPIDConstants.P), logPrefix + ":m_drivingPIDController.setP");
    Utils.setConfiguration(m_drivingPIDController.setI(Constants.Drive.SwerveModule.kDrivingMotorPIDConstants.I), logPrefix + ":m_drivingPIDController.setI");
    Utils.setConfiguration(m_drivingPIDController.setD(Constants.Drive.SwerveModule.kDrivingMotorPIDConstants.D), logPrefix + ":m_drivingPIDController.setD");
    Utils.setConfiguration(m_drivingPIDController.setFF(Constants.Drive.SwerveModule.kDrivingMotorPIDConstants.FF), logPrefix + ":m_drivingPIDController.setFF");
    Utils.setConfiguration(m_drivingPIDController.setFeedbackDevice(m_drivingEncoder), logPrefix + ":m_drivingPIDController.setFeedbackDevice");
    Utils.setConfiguration(m_drivingMotor.setIdleMode(Constants.Drive.SwerveModule.kDrivingMotorIdleMode), logPrefix + ":m_drivingSparkFlex.setIdleMode");
    Utils.setConfiguration(m_drivingMotor.setSmartCurrentLimit(Constants.Drive.SwerveModule.kDrivingMotorCurrentLimit), logPrefix + ":m_drivingSparkFlex.setSmartCurrentLimit");

    m_turningMotor = new CANSparkMax(turningMotorCanId, MotorType.kBrushless);
    m_motorControllers.add(m_turningMotor);
    Utils.setConfiguration(m_turningMotor.restoreFactoryDefaults(), logPrefix + ":m_turningSparkMax.restoreFactoryDefaults");
    m_turningEncoder = m_turningMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    Utils.setConfiguration(m_turningEncoder.setPositionConversionFactor(Constants.Drive.SwerveModule.kTurningEncoderPositionConversionFactor), logPrefix + ":m_turningEncoder.setPositionConversionFactor");
    Utils.setConfiguration(m_turningEncoder.setVelocityConversionFactor(Constants.Drive.SwerveModule.kTurningEncoderVelocityConversionFactor), logPrefix + ":m_turningEncoder.setVelocityConversionFactor");
    Utils.setConfiguration(m_turningEncoder.setInverted(Constants.Drive.SwerveModule.kTurningEncoderInverted), logPrefix + ":m_turningEncoder.setInverted");
    m_turningPIDController = m_turningMotor.getPIDController();
    Utils.setConfiguration(m_turningPIDController.setOutputRange(Constants.Drive.SwerveModule.kTurningMotorMinOutput, Constants.Drive.SwerveModule.kTurningMotorMaxOutput), logPrefix + ":m_turningPIDController.setOutputRange");
    Utils.setConfiguration(m_turningPIDController.setP(Constants.Drive.SwerveModule.kTurningMotorPIDConstants.P), logPrefix + ":m_turningPIDController.setP");
    Utils.setConfiguration(m_turningPIDController.setI(Constants.Drive.SwerveModule.kTurningMotorPIDConstants.I), logPrefix + ":m_turningPIDController.setI");
    Utils.setConfiguration(m_turningPIDController.setD(Constants.Drive.SwerveModule.kTurningMotorPIDConstants.D), logPrefix + ":m_turningPIDController.setD");
    Utils.setConfiguration(m_turningPIDController.setFF(Constants.Drive.SwerveModule.kTurningMotorPIDConstants.FF), logPrefix + ":m_turningPIDController.setFF");
    Utils.setConfiguration(m_turningPIDController.setPositionPIDWrappingEnabled(true), logPrefix + ":m_turningPIDController.setPositionPIDWrappingEnabled");
    Utils.setConfiguration(m_turningPIDController.setPositionPIDWrappingMinInput(Constants.Drive.SwerveModule.kTurningEncoderPositionPIDMinInput), logPrefix + ":m_turningPIDController.setPositionPIDWrappingMinInput");
    Utils.setConfiguration(m_turningPIDController.setPositionPIDWrappingMaxInput(Constants.Drive.SwerveModule.kTurningEncoderPositionPIDMaxInput), logPrefix + ":m_turningPIDController.setPositionPIDWrappingMaxInput");
    Utils.setConfiguration(m_turningPIDController.setFeedbackDevice(m_turningEncoder), logPrefix + ":m_turningPIDController.setFeedbackDevice");
    Utils.setConfiguration(m_turningMotor.setIdleMode(Constants.Drive.SwerveModule.kTurningMotorIdleMode), logPrefix + ":m_turningSparkMax.setIdleMode");
    Utils.setConfiguration(m_turningMotor.setSmartCurrentLimit(Constants.Drive.SwerveModule.kTurningMotorCurrentLimit), logPrefix + ":m_turningSparkMax.setSmartCurrentLimit");

    m_turningOffset = turningOffset;

    resetEncoders();
  }

  public void setTargetState(SwerveModuleState targetState) {
    targetState.angle = targetState.angle.plus(Rotation2d.fromRadians(m_turningOffset));
    targetState = SwerveModuleState.optimize(targetState, new Rotation2d(m_turningEncoder.getPosition()));
    targetState.speedMetersPerSecond *= targetState.angle.minus(new Rotation2d(m_turningEncoder.getPosition())).getCos();
    m_drivingPIDController.setReference(targetState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningPIDController.setReference(targetState.angle.getRadians(), ControlType.kPosition);
    m_setSpeed = targetState.speedMetersPerSecond;
  }

  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(m_drivingEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition() - m_turningOffset));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_drivingEncoder.getPosition(), new Rotation2d(m_turningEncoder.getPosition() - m_turningOffset));
  }

  public void setIdleMode(IdleMode idleMode) {
    m_drivingMotor.setIdleMode(idleMode);
    m_turningMotor.setIdleMode(idleMode);
  }

  public static void burnFlashForAllMotorControllers() {
    //Timer.delay(0.5);
    for (CANSparkBase motorController : m_motorControllers) {
      motorController.burnFlash();
      Timer.delay(0.05);
    }
    Timer.delay(0.5);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    String location = m_location.toString() + "/";
    builder.addDoubleProperty(location + "Turning/Position", () -> m_turningEncoder.getPosition(), null);
    builder.addDoubleProperty(location + "Driving/Position", () -> m_drivingEncoder.getPosition(), null);
    builder.addDoubleProperty(location + "Driving/Velocity", () -> m_drivingEncoder.getVelocity(), null);
    builder.addDoubleProperty(location + "Driving/AppliedOutput", m_drivingMotor::getAppliedOutput, null);
    builder.addDoubleProperty(location + "Driving/SetSpeed", () -> m_setSpeed, null);
  }
}
