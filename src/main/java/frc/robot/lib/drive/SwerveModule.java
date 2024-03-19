package frc.robot.lib.drive;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.common.Utils;
import frc.robot.lib.common.Enums.SwerveModuleLocation;
import frc.robot.lib.logging.Logger;
import frc.robot.Constants;

public class SwerveModule implements Sendable {
  private final SwerveModuleLocation m_location;

  private final CANSparkMax m_drivingMotor;
  private final CANSparkMax m_turningMotor;
  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;
  private final SparkPIDController m_drivingPIDController;
  private final SparkPIDController m_turningPIDController;

  private double m_turningOffset = 0;
  private double m_setSpeed = 0;

  public SwerveModule(SwerveModuleLocation location, int drivingMotorCanId, int turningMotorCanId, double turningOffset) {
    m_location = location;

    m_drivingMotor = new CANSparkMax(drivingMotorCanId, MotorType.kBrushless);
    m_drivingMotor.setCANMaxRetries(10);
    m_drivingMotor.restoreFactoryDefaults();
    m_drivingEncoder = m_drivingMotor.getEncoder();
    m_drivingPIDController = m_drivingMotor.getPIDController();
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    setDrivingMotorParams();
    m_drivingMotor.burnFlash();
    m_drivingEncoder.setPosition(0);

    m_turningMotor = new CANSparkMax(turningMotorCanId, MotorType.kBrushless);
    m_turningMotor.setCANMaxRetries(10);
    m_turningMotor.restoreFactoryDefaults();
    m_turningEncoder = m_turningMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    m_turningPIDController = m_turningMotor.getPIDController();
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);
    setTurningMotorParams();
    m_turningMotor.burnFlash();
    m_turningOffset = turningOffset;
  }

  private void setDrivingMotorParams() {
    m_drivingEncoder.setPositionConversionFactor(Constants.Drive.SwerveModule.kDrivingEncoderPositionConversionFactor);
    Utils.checkForREVSparkError(m_drivingEncoder.getPositionConversionFactor() != Constants.Drive.SwerveModule.kDrivingEncoderPositionConversionFactor);
    m_drivingEncoder.setVelocityConversionFactor(Constants.Drive.SwerveModule.kDrivingEncoderVelocityConversionFactor);
    Utils.checkForREVSparkError(m_drivingEncoder.getVelocityConversionFactor() == Constants.Drive.SwerveModule.kDrivingEncoderVelocityConversionFactor);
    m_drivingPIDController.setP(Constants.Drive.SwerveModule.kDrivingMotorPIDConstants.P());
    Utils.checkForREVSparkError(m_drivingPIDController.getP() != Constants.Drive.SwerveModule.kDrivingMotorPIDConstants.P());
    m_drivingPIDController.setI(Constants.Drive.SwerveModule.kDrivingMotorPIDConstants.I());
    m_drivingPIDController.setD(Constants.Drive.SwerveModule.kDrivingMotorPIDConstants.D());
    m_drivingPIDController.setFF(Constants.Drive.SwerveModule.kDrivingMotorPIDConstants.FF());
    m_drivingPIDController.setOutputRange(Constants.Drive.SwerveModule.kDrivingMotorMinOutput, Constants.Drive.SwerveModule.kDrivingMotorMaxOutput);
    m_drivingMotor.setIdleMode(Constants.Drive.SwerveModule.kDrivingMotorIdleMode);
    m_drivingMotor.setSmartCurrentLimit(Constants.Drive.SwerveModule.kDrivingMotorCurrentLimit);
  }

  private void setTurningMotorParams() {
    m_turningEncoder.setPositionConversionFactor(Constants.Drive.SwerveModule.kTurningEncoderPositionConversionFactor);
    Utils.checkForREVSparkError(m_turningEncoder.getPositionConversionFactor() != Constants.Drive.SwerveModule.kTurningEncoderPositionConversionFactor);
    m_turningEncoder.setVelocityConversionFactor(Constants.Drive.SwerveModule.kTurningEncoderVelocityConversionFactor);
    Utils.checkForREVSparkError(m_turningEncoder.getVelocityConversionFactor() != Constants.Drive.SwerveModule.kTurningEncoderVelocityConversionFactor);
    m_turningEncoder.setInverted(Constants.Drive.SwerveModule.kTurningEncoderInverted);
    Utils.checkForREVSparkError(m_turningEncoder.getInverted() != Constants.Drive.SwerveModule.kTurningEncoderInverted);
    m_turningPIDController.setP(Constants.Drive.SwerveModule.kTurningMotorPIDConstants.P());
    Utils.checkForREVSparkError(m_turningPIDController.getP() != Constants.Drive.SwerveModule.kTurningMotorPIDConstants.P());
    m_turningPIDController.setI(Constants.Drive.SwerveModule.kTurningMotorPIDConstants.I());
    m_turningPIDController.setD(Constants.Drive.SwerveModule.kTurningMotorPIDConstants.D());
    m_turningPIDController.setFF(Constants.Drive.SwerveModule.kTurningMotorPIDConstants.FF());
    m_turningPIDController.setOutputRange(Constants.Drive.SwerveModule.kTurningMotorMinOutput, Constants.Drive.SwerveModule.kTurningMotorMaxOutput);
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(Constants.Drive.SwerveModule.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(Constants.Drive.SwerveModule.kTurningEncoderPositionPIDMaxInput);
    m_turningMotor.setIdleMode(Constants.Drive.SwerveModule.kTurningMotorIdleMode);
    m_turningMotor.setSmartCurrentLimit(Constants.Drive.SwerveModule.kTurningMotorCurrentLimit);
  }

  public void setTargetState(SwerveModuleState targetState) {
    targetState.angle = targetState.angle.plus(Rotation2d.fromRadians(m_turningOffset));
    targetState = SwerveModuleState.optimize(targetState, new Rotation2d(m_turningEncoder.getPosition()));
    targetState.speedMetersPerSecond *= targetState.angle.minus(new Rotation2d(m_turningEncoder.getPosition())).getCos();
    m_drivingPIDController.setReference(targetState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningPIDController.setReference(targetState.angle.getRadians(), ControlType.kPosition);
    m_setSpeed = targetState.speedMetersPerSecond;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(m_drivingEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition() - m_turningOffset));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_drivingEncoder.getPosition(), new Rotation2d(m_turningEncoder.getPosition() - m_turningOffset));
  }

  public void updateTelemetry() {
    SmartDashboard.putNumber("Robot/Drive/SwerveModule/" + m_location + "/Driving/Velocity", m_drivingEncoder.getVelocity());
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
