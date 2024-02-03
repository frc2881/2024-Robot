package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.lib.Utils;
import frc.robot.lib.drive.SwerveModule;

public class DriveSubsystem extends SubsystemBase {
  public static enum Orientation { Field, Robot; }
  public static enum SpeedMode { Competition, Training; }
  public static enum LockState { Unlocked, Locked; }
  public static enum DriftCorrection { Enabled, Disabled; }

  private Supplier<Double> m_gyroHeading;
  private final SwerveModule[] m_swerveModules;
  private final PIDController m_thetaController;
  private final SlewRateLimiter m_driveInputXFilter;
  private final SlewRateLimiter m_driveInputYFilter;
  private final SlewRateLimiter m_driveInputRotFilter;

  private Orientation m_orientation = Orientation.Field;
  private SpeedMode m_speedMode = SpeedMode.Competition;
  private LockState m_lockState = LockState.Unlocked;
  private IdleMode m_idleMode = IdleMode.kBrake;
  private DriftCorrection m_driftCorrection = DriftCorrection.Enabled;
  private boolean m_isRotationLocked = false;
  private boolean m_isAlignToTargetPoseCompleted = false;

  public DriveSubsystem(Supplier<Double> gyroHeading) {
    m_gyroHeading = gyroHeading;

    m_swerveModules = new SwerveModule[] {
      new SwerveModule(
        SwerveModule.Location.FrontLeft,
        Constants.Drive.kFrontLeftDrivingCanId,
        Constants.Drive.kFrontLeftTurningCanId,
        Constants.Drive.kFrontLeftTurningOffset),
      new SwerveModule(
        SwerveModule.Location.FrontRight,
        Constants.Drive.kFrontRightDrivingCanId,
        Constants.Drive.kFrontRightTurningCanId,
        Constants.Drive.kFrontRightTurningOffset),
      new SwerveModule(
        SwerveModule.Location.RearLeft,
        Constants.Drive.kRearLeftDrivingCanId,
        Constants.Drive.kRearLeftTurningCanId,
        Constants.Drive.kRearLeftTurningOffset),
      new SwerveModule(
        SwerveModule.Location.RearRight,
        Constants.Drive.kRearRightDrivingCanId,
        Constants.Drive.kRearRightTurningCanId, 
        Constants.Drive.kRearRightTurningOffset)
    };
    SwerveModule.burnFlashForAllControllers();

    m_thetaController = new PIDController(Constants.Drive.kThetaControllerP, Constants.Drive.kThetaControllerI, Constants.Drive.kThetaControllerD);
    m_thetaController.enableContinuousInput(-180.0, 180.0);
    m_thetaController.setTolerance(Constants.Drive.kThetaControllerPositionTolerance, Constants.Drive.kThetaControllerVelocityTolerance);

    m_driveInputXFilter = new SlewRateLimiter(Constants.Controllers.kDriveInputRateLimit);
    m_driveInputYFilter = new SlewRateLimiter(Constants.Controllers.kDriveInputRateLimit);
    m_driveInputRotFilter = new SlewRateLimiter(Constants.Controllers.kDriveInputRateLimit);
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  public void setOrientation(Orientation orientation) {
    m_orientation = orientation;
  }

  public void setSpeedMode(DriveSubsystem.SpeedMode speedMode) {
    m_speedMode = speedMode;
  }

  public void setDriftCorrection(DriftCorrection driftCorrection) {
    m_driftCorrection = driftCorrection;
  }

  public Command driveWithControllerCommand(Supplier<Double> controllerLeftY, Supplier<Double> controllerLeftX, Supplier<Double> controllerRightX) {
    return Commands.run(
      () -> {
        if (m_lockState == LockState.Locked) { return; }

        double speedX = Utils.squareInput(-controllerLeftY.get(), Constants.Controllers.kDriveInputDeadband);
        double speedY = Utils.squareInput(-controllerLeftX.get(), Constants.Controllers.kDriveInputDeadband);
        double rotation = Utils.squareInput(-controllerRightX.get(), Constants.Controllers.kDriveInputDeadband);
        
        if (m_speedMode == SpeedMode.Training) {
          speedX = m_driveInputXFilter.calculate(speedX * Constants.Controllers.kDriveInputLimiter);
          speedY = m_driveInputYFilter.calculate(speedY * Constants.Controllers.kDriveInputLimiter);
          rotation = m_driveInputRotFilter.calculate(rotation * Constants.Controllers.kDriveInputLimiter);
        }

        if (m_driftCorrection == DriftCorrection.Enabled) {
          boolean isRotating = (rotation != 0.0);
          boolean isTranslating = ((speedX != 0.0) || (speedY != 0.0));
          if (!m_isRotationLocked && !isRotating && isTranslating) {
            m_isRotationLocked = true;
            m_thetaController.reset();
            m_thetaController.setSetpoint(m_gyroHeading.get());
          } else if (isRotating || !isTranslating) {
            m_isRotationLocked = false;
          }
          if (m_isRotationLocked) {
            rotation = m_thetaController.calculate(m_gyroHeading.get());
            if (m_thetaController.atSetpoint()) {
              rotation = 0.0;
            }
          }
        }

        drive(speedX, speedY, rotation);
      }, 
      this)
      .withName("DriveWithController");
  }

  public void drive(double speedX, double speedY, double rotation) {
    speedX *= Constants.Drive.kMaxSpeedMetersPerSecond;
    speedY *= Constants.Drive.kMaxSpeedMetersPerSecond;
    rotation *= Constants.Drive.kMaxAngularSpeed;
    drive((m_orientation == Orientation.Field)
      ? ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, rotation, Rotation2d.fromDegrees(m_gyroHeading.get()))
      : new ChassisSpeeds(speedX, speedY, rotation)
    );
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    setSwerveModuleStates(
      Constants.Drive.kSwerveDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(chassisSpeeds, 0.02)
      )
    );
  }

  public Command alignToTargetPose(Pose2d targetPose, Supplier<Pose2d> currentPose) {
    return Commands.run(
      () -> {
        Pose2d pose = currentPose.get();
        Rotation2d currentRotation = pose.getRotation();

        Transform2d delta = targetPose.minus(pose);
        Double targetAngle = Rotation2d.fromRadians(Math.atan(delta.getY()/delta.getX())).getDegrees();
        
        m_thetaController.setSetpoint(targetAngle);
        
        double rotationVel = m_thetaController.calculate(currentRotation.getDegrees());
        rotationVel += Math.copySign(0.15, rotationVel); 

        //double pidMaxSpeed = 1.0;

        // double yVel = MathUtil.clamp(
        //       m_yController.calculate((delta.getY() * 10)), 
        //       -pidMaxSpeed, 
        //       pidMaxSpeed);

        // double xVel = MathUtil.clamp(
        //       m_xController.calculate((delta.getX() * 10)), 
        //       -pidMaxSpeed, 
        //       pidMaxSpeed);
      
        if (m_thetaController.atSetpoint()) {
          rotationVel = 0.0;
        }

        // if (m_yController.atSetpoint()){
        //   yVel = 0.0;
        // }

        // if (m_xController.atSetpoint()){
        //   xVel = 0.0;
        // }

        setSwerveModuleStates(convertToSwerveModuleStates(
            0.0, //-xVel,
            0.0, //-yVel, 
            rotationVel));

        m_isAlignToTargetPoseCompleted = rotationVel <= 0.1; // && xVel <= 0.1 && yVel <= 0.1
      }, this)
      .until(() -> m_isAlignToTargetPoseCompleted)
      .withName("AlignToTargetPose");
  }

  public ChassisSpeeds getSpeeds() {
    return Constants.Drive.kSwerveDriveKinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
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

  public void setSwerveModuleStates(SwerveModuleState[] swerveModuleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Drive.kMaxSpeedMetersPerSecond);
    for (int i = 0; i < m_swerveModules.length; i++) {
      m_swerveModules[i].setTargetState(swerveModuleStates[i]);
    }
  }

  public SwerveModuleState[] convertToSwerveModuleStates(double xTranslation, double yTranslation, double rotation) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xTranslation, yTranslation, rotation);
    SwerveModuleState[] moduleStates = Constants.Drive.kSwerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    return moduleStates;
  }

  public Command toggleLockStateCommand() {
    return Commands.runOnce(
      () -> {
        setLockState((m_lockState == LockState.Unlocked) ? LockState.Locked : LockState.Unlocked);
      })
      .withName("ToggleDriveLockState");
  }

  public void setLockState(LockState lockState) {
    m_lockState = lockState;
    if (m_lockState == LockState.Locked) {
      setSwerveModuleStatesToLocked();
    }
  }

  private void setSwerveModuleStatesToLocked() {
    for (int i = 0; i < m_swerveModules.length; i++) {
      m_swerveModules[i].setTargetState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(( i == 0 || i == 3) ? 45 : -45))
      );
    }
  }

  public void setIdleMode(IdleMode idleMode) {
    m_idleMode = idleMode;
    for (int i = 0; i < m_swerveModules.length; i++) {
      m_swerveModules[i].setIdleMode(m_idleMode);
    }
  }

  private void updateTelemetry() {
    SmartDashboard.putString("Robot/Drive/LockState", m_lockState.toString());
    SmartDashboard.putString("Robot/Drive/IdleMode", m_idleMode.toString().toUpperCase().substring(1));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder); 
    for (int i = 0; i < m_swerveModules.length; i++) {
      m_swerveModules[i].initSendable(builder);
    }
  }
}
