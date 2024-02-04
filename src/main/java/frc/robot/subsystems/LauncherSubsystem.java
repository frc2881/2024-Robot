package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {
  private final CANSparkMax m_topRoller;
  private final CANSparkMax m_bottomRoller;
  private final CANSparkMax m_leadScrewMotor;
  private final SparkPIDController m_leadScrewPID;
  private final RelativeEncoder m_leadScrewEncoder;

  private double m_velocity = (33.0 / Constants.Launcher.kRotationsToInches) * 60;
  private double m_acceleration = (100.0 / Constants.Launcher.kVelocityConversion);

  private boolean m_launchInSpeaker = true;

  // 3 motors, 2 for rollers, 1 for motor to run 2 lead screw
  // Don't need to switch direction on rollers, need position control for lead screw
  // Add command to reset lead screw (connect to arm reset of lead screw?)

  public LauncherSubsystem() {
    m_leadScrewMotor = new CANSparkMax(Constants.Launcher.kLeadScrewCanId, MotorType.kBrushless);
    m_leadScrewMotor.restoreFactoryDefaults();
    m_leadScrewMotor.setIdleMode(IdleMode.kBrake); 
    m_leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_leadScrewMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)Constants.Launcher.kLeadScrewForwardLimit); 
    m_leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_leadScrewMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float)Constants.Launcher.kLeadScrewReverseLimit);
    m_leadScrewMotor.setSmartCurrentLimit(60);
    m_leadScrewMotor.setSecondaryCurrentLimit(60, 0);

    m_topRoller = new CANSparkMax(Constants.Launcher.kTopRollerCanId, MotorType.kBrushless);
    m_bottomRoller = new CANSparkMax(Constants.Launcher.kBottomRollerCanId, MotorType.kBrushless);

    m_leadScrewEncoder = m_leadScrewMotor.getEncoder();
    m_leadScrewEncoder.setPositionConversionFactor(Constants.Launcher.kRotationsToInches);
    m_leadScrewEncoder.setVelocityConversionFactor(Constants.Launcher.kVelocityConversion);
    
    m_leadScrewPID = m_leadScrewMotor.getPIDController();
    m_leadScrewPID.setSmartMotionMaxAccel(m_acceleration, 0);
    m_leadScrewPID.setFeedbackDevice(m_leadScrewEncoder);
    m_leadScrewPID.setP(Constants.Launcher.kLeadScrewP);
    m_leadScrewPID.setD(Constants.Launcher.kLeadScrewD);
    m_leadScrewPID.setOutputRange(Constants.Launcher.kLeadScrewMinOutput,
                                  Constants.Launcher.kLeadScrewMaxOutput);
  }

    @Override
  public void periodic() {
    updateTelemetry();
  }

  public Command runLeadScrewCommand(double speed) { 
    return Commands.run(
      () -> m_leadScrewMotor.set(speed), 
      this)
      .withName("RunLeadScrew"); // TODO: Change name?
  }

  public Command runLeadScrewCommand(Supplier<Double> speeds) { 
    return Commands.run(
      () -> m_leadScrewMotor.set(speeds.get()), // TODO: Make negative?
      this)
      .withName("RunLeadScrew"); // TODO: Change name??
  }

  public Command runRollersCommand(double upperSpeed, double lowerSpeed) {
    return Commands.parallel(
      Commands.run(
        () -> m_topRoller.set(upperSpeed), 
        this),
      Commands.run(
        () -> m_bottomRoller.set(lowerSpeed), 
        this)
    )
    .withName("RunRollers");
  }

  public void setDesiredPosition(double position, double speed) {
    speed *= m_velocity;
    m_leadScrewPID.setSmartMotionMaxVelocity(speed, 0);
    m_leadScrewPID.setReference(position, CANSparkMax.ControlType.kSmartMotion);
  }

  public void setDesiredPosition(double position) {
    m_leadScrewPID.setSmartMotionMaxVelocity(m_velocity, 0);
    m_leadScrewPID.setReference(position, CANSparkMax.ControlType.kSmartMotion);
  }

  // In inches
  public double getEncoderPosition() {
    return m_leadScrewEncoder.getPosition();
  }

  public void resetEncoder() {
    m_leadScrewEncoder.setPosition(0);
  }

  public void enableSoftLimitsCommand(boolean enable){
    if (enable) {
      m_leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
      m_leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    } else {
      m_leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
      m_leadScrewMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
    }
  }

  public void toggleTarget() {
    m_launchInSpeaker = !m_launchInSpeaker;
  }

  public Command tiltLauncherOverrideCommand() {
    return Commands.runOnce(
      () -> {
        enableSoftLimitsCommand(false);
      }
    )
    .andThen(
      runLeadScrewCommand(-0.15)
    )
    .finallyDo(
      () -> {
        enableSoftLimitsCommand(true);
        resetEncoder();
        runLeadScrewCommand(0.0);
      }
    )
    .withName("tiltLauncherOverride");
  }

  public Command tiltLauncherToHeightCommand(double speed, double position) {
    return Commands.run(
      () -> {
        setDesiredPosition(position, speed);
      }
    )
    .until(() -> (Math.abs(getEncoderPosition() - position) < 0.1))
    .andThen(runLeadScrewCommand(0.0))
    .withName("tiltLauncherToHeight");
  }

  public Double findLaunchAngle(Pose2d pose){
    if(m_launchInSpeaker){
      Double height = Constants.Game.Field.Targets.kBlueSpeaker.getZ();

      // get dist diagonally from speaker
      // use dist/height to get ang
      // use ang + dist from pivot points to get height of lead screw

      // add logic for launch zone
      // figure out how to decide to shoot for speaker/amp
      return 0.0;
    } 

    Double height = Constants.Game.Field.Targets.kBlueAmp.getZ();

    // get dist diagonally from speaker
    // use dist/height to get ang
    // use ang + dist from pivot points to get height of lead screw

    // add logic for launch zone
    // figure out how to decide to shoot for speaker/amp
    return 0.0;
    
  }

  public Command updateLaunchAngle(Pose2d pose){
    // add logic for launch zone
    return tiltLauncherToHeightCommand(Constants.Launcher.kLaunchUpdateSpeed, findLaunchAngle(pose));
  }

  private void updateTelemetry() {
    // TODO: send subsystem telemetry data to the dashboard as needed
    // ex: SmartDashboard.putString("Robot/Example/String", "TEST");
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // TODO: send subsystem data to be logged on the robot as needed
    // ex: builder.addDoubleProperty("Double", this::getSomeDoubleValue, null);
  }
}
