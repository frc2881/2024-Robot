package frc.robot.lib.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.common.Enums.SwerveModuleLocation;
import frc.robot.lib.common.Utils;
import frc.robot.Constants;

public class SwerveModule implements Sendable {
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

    m_drivingMotor = new CANSparkFlex(drivingMotorCanId, MotorType.kBrushless);
    m_drivingMotor.setCANMaxRetries(10);
    Utils.validateREVLib(m_drivingMotor.restoreFactoryDefaults());
    m_drivingEncoder = m_drivingMotor.getEncoder();
    m_drivingPIDController = m_drivingMotor.getPIDController();
    Utils.validateREVLib(m_drivingPIDController.setFeedbackDevice(m_drivingEncoder));
    Utils.validateREVLib(m_drivingEncoder.setPositionConversionFactor(Constants.Drive.SwerveModule.kDrivingEncoderPositionConversionFactor));
    Utils.validateREVLib(m_drivingEncoder.setVelocityConversionFactor(Constants.Drive.SwerveModule.kDrivingEncoderVelocityConversionFactor));
    Utils.validateREVLib(m_drivingPIDController.setP(Constants.Drive.SwerveModule.kDrivingMotorPIDConstants.P()));
    Utils.validateREVLib(m_drivingPIDController.setI(Constants.Drive.SwerveModule.kDrivingMotorPIDConstants.I()));
    Utils.validateREVLib(m_drivingPIDController.setD(Constants.Drive.SwerveModule.kDrivingMotorPIDConstants.D()));
    Utils.validateREVLib(m_drivingPIDController.setFF(Constants.Drive.SwerveModule.kDrivingMotorPIDConstants.FF()));
    Utils.validateREVLib(m_drivingPIDController.setOutputRange(Constants.Drive.SwerveModule.kDrivingMotorMaxReverseOutput, Constants.Drive.SwerveModule.kDrivingMotorMaxForwardOutput));
    Utils.validateREVLib(m_drivingMotor.setIdleMode(Constants.Drive.SwerveModule.kDrivingMotorIdleMode));
    Utils.validateREVLib(m_drivingMotor.setSmartCurrentLimit(Constants.Drive.SwerveModule.kDrivingMotorCurrentLimit));
    Utils.validateREVLib(m_drivingMotor.burnFlash());
    Utils.validateREVLib(m_drivingEncoder.setPosition(0));

    m_turningMotor = new CANSparkMax(turningMotorCanId, MotorType.kBrushless);
    m_turningMotor.setCANMaxRetries(10);
    Utils.validateREVLib(m_turningMotor.restoreFactoryDefaults());
    m_turningEncoder = m_turningMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    m_turningPIDController = m_turningMotor.getPIDController();
    Utils.validateREVLib(m_turningPIDController.setFeedbackDevice(m_turningEncoder));
    Utils.validateREVLib(m_turningEncoder.setPositionConversionFactor(Constants.Drive.SwerveModule.kTurningEncoderPositionConversionFactor));
    Utils.validateREVLib(m_turningEncoder.setVelocityConversionFactor(Constants.Drive.SwerveModule.kTurningEncoderVelocityConversionFactor));
    Utils.validateREVLib(m_turningEncoder.setInverted(Constants.Drive.SwerveModule.kTurningEncoderInverted));
    Utils.validateREVLib(m_turningPIDController.setP(Constants.Drive.SwerveModule.kTurningMotorPIDConstants.P()));
    Utils.validateREVLib(m_turningPIDController.setI(Constants.Drive.SwerveModule.kTurningMotorPIDConstants.I()));
    Utils.validateREVLib(m_turningPIDController.setD(Constants.Drive.SwerveModule.kTurningMotorPIDConstants.D()));
    Utils.validateREVLib(m_turningPIDController.setFF(Constants.Drive.SwerveModule.kTurningMotorPIDConstants.FF()));
    Utils.validateREVLib(m_turningPIDController.setOutputRange(Constants.Drive.SwerveModule.kTurningMotorMaxReverseOutput, Constants.Drive.SwerveModule.kTurningMotorMaxForwardOutput));
    Utils.validateREVLib(m_turningPIDController.setPositionPIDWrappingEnabled(true));
    Utils.validateREVLib(m_turningPIDController.setPositionPIDWrappingMinInput(Constants.Drive.SwerveModule.kTurningEncoderPositionPIDMinInput));
    Utils.validateREVLib(m_turningPIDController.setPositionPIDWrappingMaxInput(Constants.Drive.SwerveModule.kTurningEncoderPositionPIDMaxInput));
    Utils.validateREVLib(m_turningMotor.setIdleMode(Constants.Drive.SwerveModule.kTurningMotorIdleMode));
    Utils.validateREVLib(m_turningMotor.setSmartCurrentLimit(Constants.Drive.SwerveModule.kTurningMotorCurrentLimit));
    Utils.validateREVLib(m_turningMotor.burnFlash());
    m_turningOffset = turningOffset;
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
    SmartDashboard.putNumber("Robot/Drive/SwerveModule/" + m_location + "/Driving/Speed/Target", m_setSpeed);
    SmartDashboard.putNumber("Robot/Drive/SwerveModule/" + m_location + "/Driving/Speed/Actual", m_drivingEncoder.getVelocity() * Constants.Drive.SwerveModule.kDrivingEncoderVelocityConversionFactor);
    SmartDashboard.putNumber("Robot/Drive/SwerveModule/" + m_location + "/Turning/Position", m_turningEncoder.getPosition());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    String location = m_location.toString() + "/";
    builder.addDoubleProperty(location + "Driving/Position", m_drivingEncoder::getPosition, null);
    builder.addDoubleProperty(location + "Driving/Velocity", m_drivingEncoder::getVelocity, null);
    builder.addDoubleProperty(location + "Driving/AppliedOutput", m_drivingMotor::getAppliedOutput, null);
    builder.addDoubleProperty(location + "Driving/SetSpeed", () -> m_setSpeed, null);
    builder.addDoubleProperty(location + "Turning/Position", m_turningEncoder::getPosition, null);
  }
}
