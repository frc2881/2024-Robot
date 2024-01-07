package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.lib.Utils;
import frc.robot.lib.drive.SwerveModule;
import frc.robot.lib.logging.Logger;
import frc.robot.lib.sensors.Gyro;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  public static enum Orientation { FIELD, ROBOT; }
  public static enum SpeedMode { COMPETITION, TRAINING; }
  public static enum LockState { UNLOCKED, LOCKED; }

  private final Gyro m_gyro;
  private final SwerveModule[] m_swerveModules;
  private final SwerveDriveKinematics m_swerveDriveKinematics;
  private final SwerveDrivePoseEstimator m_poseEstimator;
  private final PIDController m_thetaController;
  private final SlewRateLimiter m_driveInputXFilter;
  private final SlewRateLimiter m_driveInputYFilter;
  private final SlewRateLimiter m_driveInputRotFilter;

  private Orientation m_orientation = Orientation.FIELD;
  private SpeedMode m_speedMode = SpeedMode.COMPETITION;
  private LockState m_lockState = LockState.UNLOCKED;
  private IdleMode m_idleMode = IdleMode.kBrake;
  private boolean m_isRotationLocked = false;

  public DriveSubsystem(Gyro gyro) {
    m_gyro = gyro;

    m_swerveModules = new SwerveModule[] {
      new SwerveModule(
        SwerveModule.Location.FrontLeft,
        Constants.Drive.kFrontLeftDrivingCanId,
        Constants.Drive.kFrontLeftTurningCanId,
        Constants.Drive.kFrontLeftChassisAngularOffset),
      new SwerveModule(
        SwerveModule.Location.FrontRight,
        Constants.Drive.kFrontRightDrivingCanId,
        Constants.Drive.kFrontRightTurningCanId,
        Constants.Drive.kFrontRightChassisAngularOffset),
      new SwerveModule(
        SwerveModule.Location.RearLeft,
        Constants.Drive.kRearLeftDrivingCanId,
        Constants.Drive.kRearLeftTurningCanId,
        Constants.Drive.kRearLeftChassisAngularOffset),
      new SwerveModule(
        SwerveModule.Location.RearRight,
        Constants.Drive.kRearRightDrivingCanId,
        Constants.Drive.kRearRightTurningCanId, 
        Constants.Drive.kRearRightChassisAngularOffset)
    };

    m_swerveDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.Drive.kWheelBase / 2, Constants.Drive.kTrackWidth / 2),
      new Translation2d(Constants.Drive.kWheelBase / 2, -Constants.Drive.kTrackWidth / 2),
      new Translation2d(-Constants.Drive.kWheelBase / 2, Constants.Drive.kTrackWidth / 2),
      new Translation2d(-Constants.Drive.kWheelBase / 2, -Constants.Drive.kTrackWidth / 2)
    );

    m_poseEstimator = new SwerveDrivePoseEstimator(
      m_swerveDriveKinematics, 
      m_gyro.getRotation2d(), 
      getSwerveModulePositions(), 
      new Pose2d());

    m_thetaController = new PIDController(Constants.Drive.kThetaControllerP, Constants.Drive.kThetaControllerI, Constants.Drive.kThetaControllerD);
    m_thetaController.enableContinuousInput(-180.0, 180.0);
    m_thetaController.setTolerance(Constants.Drive.kThetaControllerPositionTolerance, Constants.Drive.kThetaControllerVelocityTolerance);

    m_driveInputXFilter = new SlewRateLimiter(Constants.Controllers.kDriveInputRateLimit);
    m_driveInputYFilter = new SlewRateLimiter(Constants.Controllers.kDriveInputRateLimit);
    m_driveInputRotFilter = new SlewRateLimiter(Constants.Controllers.kDriveInputRateLimit);

    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetPose, 
      this::getSpeeds, 
      this::drive, 
      new HolonomicPathFollowerConfig(
        new PIDConstants(Constants.Drive.kPathFollowerTranslationP, Constants.Drive.kPathFollowerTranslationI, Constants.Drive.kPathFollowerTranslationD),
        new PIDConstants(Constants.Drive.kPathFollowerRotationP, Constants.Drive.kPathFollowerRotationI, Constants.Drive.kPathFollowerRotationD), 
        Constants.Drive.kMaxSpeedMetersPerSecond, 
        Constants.Drive.kDriveBaseRadius, 
        new ReplanningConfig()
      ), 
      this
    );
  }

  public Command driveWithControllerCommand(XboxController controller) {
    return Commands.run(
      () -> {
        if (m_lockState == LockState.LOCKED) { return; }

        double speedX = Utils.squareInput(-controller.getLeftY(), Constants.Controllers.kDriveInputDeadband);
        double speedY = Utils.squareInput(-controller.getLeftX(), Constants.Controllers.kDriveInputDeadband);
        double rotation = Utils.squareInput(-controller.getRightX(), Constants.Controllers.kDriveInputDeadband);
        
        if (m_speedMode == SpeedMode.TRAINING) {
          speedX = m_driveInputXFilter.calculate(speedX * Constants.Controllers.kDriveInputLimiter);
          speedY = m_driveInputYFilter.calculate(speedY * Constants.Controllers.kDriveInputLimiter);
          rotation = m_driveInputRotFilter.calculate(rotation * Constants.Controllers.kDriveInputLimiter);
        }

        boolean isRotating = (rotation != 0.0);
        boolean isTranslating = ((speedX != 0.0) || (speedY != 0.0));
        if (!m_isRotationLocked && !isRotating && isTranslating) {
          m_isRotationLocked = true;
          m_thetaController.reset();
          m_thetaController.setSetpoint(m_gyro.getHeading());
        } else if (isRotating || !isTranslating) {
          m_isRotationLocked = false;
        }
        if (m_isRotationLocked) {
          rotation = m_thetaController.calculate(m_gyro.getHeading());
          if (m_thetaController.atSetpoint()) {
            rotation = 0.0;
          }
        }

        this.drive(speedX, speedY, rotation);
      }, 
      this)
      .withName("DriveWithController");
  }

  public Command toggleLockStateCommand() {
    return Commands.runOnce(
      () -> {
        m_lockState = (m_lockState == LockState.UNLOCKED) ? LockState.LOCKED : LockState.UNLOCKED; 
        if (m_lockState == LockState.LOCKED) {
          setSwerveModuleStatesToLocked();
        }
      })
      .withName("ToggleDriveLockState");
  }

  public Command setForCalibrationCommand() {
    return Commands.runOnce(
      () -> { 
        m_lockState = LockState.LOCKED;
        setSwerveModuleStatesForCalibration(); 
      })
      .withName("SetForCalibration");
  }

  public void drive(double speedX, double speedY, double rotation) {
    speedX *= Constants.Drive.kMaxSpeedMetersPerSecond;
    speedY *= Constants.Drive.kMaxSpeedMetersPerSecond;
    rotation *= Constants.Drive.kMaxAngularSpeed;
    drive((m_orientation == Orientation.FIELD)
      ? ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, rotation, Rotation2d.fromDegrees(m_gyro.getAngle()))
      : new ChassisSpeeds(speedX, speedY, rotation)
    );
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    setSwerveModuleStates(
      m_swerveDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(chassisSpeeds, 0.02)
      )
    );
  }

  @Override
  public void periodic() {
    updatePose();
    updateTelemetry();
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void updatePose() {
    m_poseEstimator.update(m_gyro.getRotation2d(), getSwerveModulePositions());
  }

  public void resetPose() {
    resetPose(new Pose2d());
  }

  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(m_gyro.getRotation2d(), getSwerveModulePositions(), pose);
    Logger.log("Pose reset to: " + pose.toString());
  }

  public void setOrientation(Orientation orientation) {
    m_orientation = orientation;
  }

  public void setSpeedMode(DriveSubsystem.SpeedMode speedMode) {
    m_speedMode = speedMode;
  }

  public ChassisSpeeds getSpeeds() {
    return m_swerveDriveKinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  private SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[m_swerveModules.length];
    for (int i = 0; i < m_swerveModules.length; i++) {
      positions[i] = m_swerveModules[i].getPosition();
    }
    return positions;
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[m_swerveModules.length];
    for (int i = 0; i < m_swerveModules.length; i++) {
      states[i] = m_swerveModules[i].getState();
    }
    return states;
  }

  public SwerveModuleState[] convertToSwerveModuleStates(double xTranslation, double yTranslation, double rotation) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xTranslation, yTranslation, rotation);
    SwerveModuleState[] moduleStates = m_swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    return moduleStates;
  }

  public void setSwerveModuleStates(SwerveModuleState[] swerveModuleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Drive.kMaxSpeedMetersPerSecond);
    for (int i = 0; i < m_swerveModules.length; i++) {
      m_swerveModules[i].setTargetState(swerveModuleStates[i]);
    }
  }

  private void setSwerveModuleStatesToLocked() {
    for (int i = 0; i < m_swerveModules.length; i++) {
      m_swerveModules[i].setTargetState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(( i == 0 || i == 3) ? 45 : -45))
      );
    }
  }

  private void setSwerveModuleStatesForCalibration() {
    for (int i = 0; i < m_swerveModules.length; i++) {
      m_swerveModules[i].setTargetState(
        new SwerveModuleState(0, Rotation2d.fromRadians(m_swerveModules[i].getChassisAngularOffset()))
      );
    }
  }

  public void setIdleMode(IdleMode idleMode) {
    m_idleMode = idleMode;
    for (int i = 0; i < m_swerveModules.length; i++) {
      m_swerveModules[i].setIdleMode(m_idleMode);
    }
  }

  public void resetEncoders() {
    for (int i = 0; i < m_swerveModules.length; i++) {
      m_swerveModules[i].resetEncoders();
    }
  }

  public void reset() {
    resetEncoders();
    m_lockState = LockState.UNLOCKED; 
  }

  private void updateTelemetry() {
    m_gyro.updateTelemetry();
    SmartDashboard.putString("Robot/Drive/LockState", m_lockState.toString());
    SmartDashboard.putString("Robot/Drive/IdleMode", m_idleMode.toString().toUpperCase().substring(1));
    SmartDashboard.putString("Robot/Drive/Pose", Utils.objectToJson(getPose()));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    m_gyro.initSendable(builder); 
    for (int i = 0; i < m_swerveModules.length; i++) {
      m_swerveModules[i].initSendable(builder);
    }
  }
}
