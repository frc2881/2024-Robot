package frc.robot.lib.drive;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
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

import frc.robot.lib.logging.Logger;
import frc.robot.lib.common.Enums.SwerveModuleLocation;
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
    m_turningMotor = new CANSparkMax(turningMotorCanId, MotorType.kBrushless);

    setParam(m_drivingMotor.restoreFactoryDefaults(), "m_drivingSparkFlex.restoreFactoryDefaults");
    setParam(m_turningMotor.restoreFactoryDefaults(), "m_turningSparkMax.restoreFactoryDefaults");

    m_drivingEncoder = m_drivingMotor.getEncoder();
    m_turningEncoder = m_turningMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    m_drivingPIDController = m_drivingMotor.getPIDController();
    m_turningPIDController = m_turningMotor.getPIDController();

    setParam(m_drivingPIDController.setFeedbackDevice(m_drivingEncoder), "m_drivingPIDController.setFeedbackDevice");
    setParam(m_turningPIDController.setFeedbackDevice(m_turningEncoder), "m_turningPIDController.setFeedbackDevice");

    setParam(m_drivingEncoder.setPositionConversionFactor(Constants.Drive.SwerveModule.kDrivingEncoderPositionConversionFactor), "m_drivingEncoder.setPositionConversionFactor");
    setParam(m_drivingEncoder.setVelocityConversionFactor(Constants.Drive.SwerveModule.kDrivingEncoderVelocityConversionFactor), "m_drivingEncoder.setVelocityConversionFactor");
    setParam(m_turningEncoder.setPositionConversionFactor(Constants.Drive.SwerveModule.kTurningEncoderPositionConversionFactor), "m_turningEncoder.setPositionConversionFactor");
    setParam(m_turningEncoder.setVelocityConversionFactor(Constants.Drive.SwerveModule.kTurningEncoderVelocityConversionFactor), "m_turningEncoder.setVelocityConversionFactor");

    setParam(m_turningEncoder.setInverted(Constants.Drive.SwerveModule.kTurningEncoderInverted), "m_turningEncoder.setInverted");

    setParam(m_turningPIDController.setPositionPIDWrappingEnabled(true), "m_turningPIDController.setPositionPIDWrappingEnabled");
    setParam(m_turningPIDController.setPositionPIDWrappingMinInput(Constants.Drive.SwerveModule.kTurningEncoderPositionPIDMinInput), "m_turningPIDController.setPositionPIDWrappingMinInput");
    setParam(m_turningPIDController.setPositionPIDWrappingMaxInput(Constants.Drive.SwerveModule.kTurningEncoderPositionPIDMaxInput), "m_turningPIDController.setPositionPIDWrappingMaxInput");

    setParam(m_drivingPIDController.setP(Constants.Drive.SwerveModule.kDrivingMotorPIDConstants.P), "m_drivingPIDController.setP");
    setParam(m_drivingPIDController.setI(Constants.Drive.SwerveModule.kDrivingMotorPIDConstants.I), "m_drivingPIDController.setI");
    setParam(m_drivingPIDController.setD(Constants.Drive.SwerveModule.kDrivingMotorPIDConstants.D), "m_drivingPIDController.setD");
    setParam(m_drivingPIDController.setFF(Constants.Drive.SwerveModule.kDrivingMotorPIDConstants.FF), "m_drivingPIDController.setFF");
    setParam(m_drivingPIDController.setOutputRange(Constants.Drive.SwerveModule.kDrivingMotorMinOutput, Constants.Drive.SwerveModule.kDrivingMotorMaxOutput), "m_drivingPIDController.setOutputRange");

    setParam(m_turningPIDController.setP(Constants.Drive.SwerveModule.kTurningMotorPIDConstants.P), "m_turningPIDController.setP");
    setParam(m_turningPIDController.setI(Constants.Drive.SwerveModule.kTurningMotorPIDConstants.I), "m_turningPIDController.setI");
    setParam(m_turningPIDController.setD(Constants.Drive.SwerveModule.kTurningMotorPIDConstants.D), "m_turningPIDController.setD");
    setParam(m_turningPIDController.setFF(Constants.Drive.SwerveModule.kTurningMotorPIDConstants.FF), "m_turningPIDController.setFF");
    setParam(m_turningPIDController.setOutputRange(Constants.Drive.SwerveModule.kTurningMotorMinOutput, Constants.Drive.SwerveModule.kTurningMotorMaxOutput), "m_turningPIDController.setOutputRange");
    
    setParam(m_drivingMotor.setIdleMode(Constants.Drive.SwerveModule.kDrivingMotorIdleMode), "m_drivingSparkFlex.setIdleMode");
    setParam(m_turningMotor.setIdleMode(Constants.Drive.SwerveModule.kTurningMotorIdleMode), "m_turningSparkMax.setIdleMode");

    setParam(m_drivingMotor.setSmartCurrentLimit(Constants.Drive.SwerveModule.kDrivingMotorCurrentLimit), "m_drivingSparkFlex.setSmartCurrentLimit");
    setParam(m_turningMotor.setSmartCurrentLimit(Constants.Drive.SwerveModule.kTurningMotorCurrentLimit), "m_turningSparkMax.setSmartCurrentLimit");

    m_drivingMotor.burnFlash();
    m_turningMotor.burnFlash();

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

  private void setParam(REVLibError error, String source) {
    if (error != REVLibError.kOk) {
      Logger.log("!!!!!!!!!! REVLibError Returned: " + error.toString() + " @ SwerveModule:" + m_location.toString() + ":" + source + " !!!!!!!!!!");
    }
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
