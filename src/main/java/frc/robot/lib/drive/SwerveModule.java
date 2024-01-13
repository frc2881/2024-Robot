package frc.robot.lib.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
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
import frc.robot.Constants;

public class SwerveModule implements Sendable {

  public static enum Location { FrontLeft, FrontRight, RearLeft, RearRight; }

  private final Location m_location;

  private final CANSparkFlex m_drivingSparkFlex;
  private final CANSparkMax m_turningSparkMax;
  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;
  private final SparkPIDController m_drivingPIDController;
  private final SparkPIDController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private double m_setSpeed = 0;

  public SwerveModule(Location location, int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_location = location;

    m_drivingSparkFlex = new CANSparkFlex(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    m_drivingSparkFlex.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    m_drivingEncoder = m_drivingSparkFlex.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    m_drivingPIDController = m_drivingSparkFlex.getPIDController();
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // TODO: wait for RevLib update to fix these methods
    //m_drivingEncoder.setPositionConversionFactor(Constants.Drive.SwerveModule.kDrivingEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(Constants.Drive.SwerveModule.kDrivingEncoderVelocityFactor);

    m_turningEncoder.setPositionConversionFactor(Constants.Drive.SwerveModule.kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(Constants.Drive.SwerveModule.kTurningEncoderVelocityFactor);

    m_turningEncoder.setInverted(Constants.Drive.SwerveModule.kTurningEncoderInverted);

    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(Constants.Drive.SwerveModule.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(Constants.Drive.SwerveModule.kTurningEncoderPositionPIDMaxInput);

    m_drivingPIDController.setP(Constants.Drive.SwerveModule.kDrivingP);
    m_drivingPIDController.setI(Constants.Drive.SwerveModule.kDrivingI);
    m_drivingPIDController.setD(Constants.Drive.SwerveModule.kDrivingD);
    m_drivingPIDController.setFF(Constants.Drive.SwerveModule.kDrivingFF);
    m_drivingPIDController.setOutputRange(Constants.Drive.SwerveModule.kDrivingMinOutput, Constants.Drive.SwerveModule.kDrivingMaxOutput);

    m_turningPIDController.setP(Constants.Drive.SwerveModule.kTurningP);
    m_turningPIDController.setI(Constants.Drive.SwerveModule.kTurningI);
    m_turningPIDController.setD(Constants.Drive.SwerveModule.kTurningD);
    m_turningPIDController.setFF(Constants.Drive.SwerveModule.kTurningFF);
    m_turningPIDController.setOutputRange(Constants.Drive.SwerveModule.kTurningMinOutput, Constants.Drive.SwerveModule.kTurningMaxOutput);

    m_drivingSparkFlex.setIdleMode(Constants.Drive.SwerveModule.kDrivingMotorIdleMode);
    m_turningSparkMax.setIdleMode(Constants.Drive.SwerveModule.kTurningMotorIdleMode);
    
    m_drivingSparkFlex.setSmartCurrentLimit(Constants.Drive.SwerveModule.kDrivingMotorCurrentLimit);
    m_turningSparkMax.setSmartCurrentLimit(Constants.Drive.SwerveModule.kTurningMotorCurrentLimit);

    m_drivingSparkFlex.burnFlash();
    m_turningSparkMax.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;

    resetEncoders();
  }

  public double getChassisAngularOffset() {
    return m_chassisAngularOffset;
  }

  public void setTargetState(SwerveModuleState targetState) {
    SwerveModuleState correctedTargetState = new SwerveModuleState();
    correctedTargetState.speedMetersPerSecond = targetState.speedMetersPerSecond;
    correctedTargetState.angle = targetState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));
    SwerveModuleState optimizedTargetState = SwerveModuleState.optimize(correctedTargetState, new Rotation2d(m_turningEncoder.getPosition()));
    m_drivingPIDController.setReference(optimizedTargetState.speedMetersPerSecond, CANSparkBase.ControlType.kVelocity);
    m_turningPIDController.setReference(optimizedTargetState.angle.getRadians(), CANSparkBase.ControlType.kPosition);
    m_setSpeed = optimizedTargetState.speedMetersPerSecond;
  }

  public void setIdleMode(IdleMode idleMode) {
    m_drivingSparkFlex.setIdleMode(idleMode);
    m_turningSparkMax.setIdleMode(idleMode);
  }

  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(m_drivingEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionScaled(), new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  public IdleMode getDrivingIdleMode() {
    return m_drivingSparkFlex.getIdleMode();
  }

  private double getPositionScaled() {
    return m_drivingEncoder.getPosition() * Constants.Drive.SwerveModule.kDrivingEncoderPositionFactor;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    String key = m_location.toString() + "/";
    builder.addDoubleProperty(key + "Turning/Position", () -> m_turningEncoder.getPosition(), null);
    builder.addDoubleProperty(key + "Driving/Position", () -> getPositionScaled(), null);
    builder.addDoubleProperty(key + "Driving/Velocity", () -> m_drivingEncoder.getVelocity(), null);
    builder.addDoubleProperty(key + "Driving/AppliedOutput", m_drivingSparkFlex::getAppliedOutput, null);
    builder.addDoubleProperty(key + "Driving/SetSpeed", () -> m_setSpeed, null);
  }
}
