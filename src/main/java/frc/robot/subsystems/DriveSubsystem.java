package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.common.Utils;
import frc.robot.lib.common.Enums.DriveDriftCorrection;
import frc.robot.lib.common.Enums.DriveLockState;
import frc.robot.lib.common.Enums.DriveOrientation;
import frc.robot.lib.common.Enums.DriveSpeedMode;
import frc.robot.lib.common.Enums.SwerveModuleLocation;
import frc.robot.lib.drive.SwerveModule;

public class DriveSubsystem extends SubsystemBase {
  private Supplier<Double> m_gyroHeading;
  private final SwerveModule[] m_swerveModules;
  private final PIDController m_driftThetaController;
  private final PIDController m_alignThetaController;
  private final SlewRateLimiter m_driveInputXFilter;
  private final SlewRateLimiter m_driveInputYFilter;
  private final SlewRateLimiter m_driveInputRotFilter;

  private DriveOrientation m_orientation = DriveOrientation.Field;
  private DriveSpeedMode m_speedMode = DriveSpeedMode.Competition;
  private DriveLockState m_lockState = DriveLockState.Unlocked;
  private IdleMode m_idleMode = IdleMode.kBrake;
  private DriveDriftCorrection m_driftCorrection = DriveDriftCorrection.Enabled;
  private boolean m_isRotationLocked = false;
  private boolean m_isAlignedToTarget = false;

  public DriveSubsystem(Supplier<Double> gyroHeading) {
    m_gyroHeading = gyroHeading;

    m_swerveModules = new SwerveModule[] {
      new SwerveModule(
        SwerveModuleLocation.FrontLeft,
        Constants.Drive.kFrontLeftDrivingMotorCANId,
        Constants.Drive.kFrontLeftTurningMotorCANId,
        Constants.Drive.SwerveModule.kOffsetFrontLeft),
      new SwerveModule(
        SwerveModuleLocation.FrontRight,
        Constants.Drive.kFrontRightDrivingMotorCANId,
        Constants.Drive.kFrontRightTurningMotorCANId,
        Constants.Drive.SwerveModule.kOffsetFrontRight),
      new SwerveModule(
        SwerveModuleLocation.RearLeft,
        Constants.Drive.kRearLeftDrivingMotorCANId,
        Constants.Drive.kRearLeftTurningMotorCANId,
        Constants.Drive.SwerveModule.kOffsetRearLeft),
      new SwerveModule(
        SwerveModuleLocation.RearRight,
        Constants.Drive.kRearRightDrivingMotorCANId,
        Constants.Drive.kRearRightTurningMotorCANId, 
        Constants.Drive.SwerveModule.kOffsetRearRight)
    };
    SwerveModule.burnFlashForAllMotorControllers();

    m_driftThetaController = new PIDController(Constants.Drive.kDriftThetaControllerPIDConstants.P, Constants.Drive.kDriftThetaControllerPIDConstants.I, Constants.Drive.kDriftThetaControllerPIDConstants.D);
    m_driftThetaController.enableContinuousInput(-180.0, 180.0);
    m_driftThetaController.setTolerance(Constants.Drive.kDriftThetaControllerPositionTolerance, Constants.Drive.kDriftThetaControllerVelocityTolerance);

    m_alignThetaController = new PIDController(Constants.Drive.kAlignThetaControllerPIDConstants.P, Constants.Drive.kAlignThetaControllerPIDConstants.I, Constants.Drive.kAlignThetaControllerPIDConstants.D);
    m_alignThetaController.enableContinuousInput(-180.0, 180.0);
    m_alignThetaController.setTolerance(Constants.Drive.kAlignThetaControllerPositionTolerance, Constants.Drive.kAlignThetaControllerVelocityTolerance);

    m_driveInputXFilter = new SlewRateLimiter(Constants.Drive.kDriveInputRateLimit);
    m_driveInputYFilter = new SlewRateLimiter(Constants.Drive.kDriveInputRateLimit);
    m_driveInputRotFilter = new SlewRateLimiter(Constants.Drive.kDriveInputRateLimit);
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  public void setOrientation(DriveOrientation orientation) {
    m_orientation = orientation;
  }

  public void setSpeedMode(DriveSpeedMode speedMode) {
    m_speedMode = speedMode;
  }

  public void setDriftCorrection(DriveDriftCorrection driftCorrection) {
    m_driftCorrection = driftCorrection;
  }

  public Command driveWithControllerCommand(Supplier<Double> controllerLeftY, Supplier<Double> controllerLeftX, Supplier<Double> controllerRightX) {
    return 
    run(() -> {
      double speedX = controllerLeftY.get();
      double speedY = controllerLeftX.get();
      double speedRotation = controllerRightX.get();
      if (m_speedMode == DriveSpeedMode.Training) {
        speedX = m_driveInputXFilter.calculate(speedX * Constants.Drive.kDriveInputLimiter);
        speedY = m_driveInputYFilter.calculate(speedY * Constants.Drive.kDriveInputLimiter);
        speedRotation = m_driveInputRotFilter.calculate(speedRotation * Constants.Drive.kDriveInputLimiter);
      }
      if (m_driftCorrection == DriveDriftCorrection.Enabled) {
        boolean isRotating = (speedRotation != 0.0);
        boolean isTranslating = ((speedX != 0.0) || (speedY != 0.0));
        if (!m_isRotationLocked && !isRotating && isTranslating) {
          m_isRotationLocked = true;
          m_driftThetaController.reset();
          m_driftThetaController.setSetpoint(m_gyroHeading.get());
        } else if (isRotating || !isTranslating) {
          m_isRotationLocked = false;
        }
        if (m_isRotationLocked) {
          speedRotation = m_driftThetaController.calculate(m_gyroHeading.get());
          if (m_driftThetaController.atSetpoint()) {
            speedRotation = 0.0;
          }
        }
      }
      drive(speedX, speedY, speedRotation);
    })
    .unless(() -> m_lockState == DriveLockState.Locked)
    .withName("DriveWithController");
  }

  public void drive(double speedX, double speedY, double speedRotation) {
    speedX *= Constants.Drive.kMaxSpeedMetersPerSecond;
    speedY *= Constants.Drive.kMaxSpeedMetersPerSecond;
    speedRotation *= Constants.Drive.kMaxAngularSpeed;
    drive((m_orientation == DriveOrientation.Field)
      ? ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, speedRotation, Rotation2d.fromDegrees(m_gyroHeading.get()))
      : new ChassisSpeeds(speedX, speedY, speedRotation)
    );
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    setSwerveModuleStates(
      Constants.Drive.kSwerveDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(chassisSpeeds, 0.02)
      )
    );
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

  public SwerveModuleState[] convertToSwerveModuleStates(double speedX, double speedY, double speedRotation) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(speedX, speedY, speedRotation);
    SwerveModuleState[] moduleStates = Constants.Drive.kSwerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    return moduleStates;
  }

  private void setSwerveModuleStatesToLocked() {
    for (int i = 0; i < m_swerveModules.length; i++) {
      m_swerveModules[i].setTargetState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(( i == 0 || i == 3) ? 45 : -45))
      );
    }
  }

  public void setLockState(DriveLockState lockState) {
    m_lockState = lockState;
    if (m_lockState == DriveLockState.Locked) {
      setSwerveModuleStatesToLocked();
    }
  }

  public Command setLockedCommand() {
    return 
    startEnd(
      () -> setLockState(DriveLockState.Locked),
      () -> setLockState(DriveLockState.Unlocked)
    )
    .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
    .withName("SetDriveLockedState");
  }

  public Command alignToTargetCommand(Supplier<Pose2d> currentPose, Pose3d targetPose) {
    return
    run(() -> {
      double speedRotation = m_alignThetaController.calculate(currentPose.get().getRotation().getDegrees());
      speedRotation += Math.copySign(0.15, speedRotation);
      if (m_alignThetaController.atSetpoint()) {
        speedRotation = 0.0;
        m_isAlignedToTarget = true;
      }
      setSwerveModuleStates(convertToSwerveModuleStates(
        0.0,
        0.0,
        speedRotation));
    })
    .beforeStarting(() -> {
      m_isAlignedToTarget = false; 
      m_alignThetaController.setSetpoint(Math.toDegrees(Utils.getTargetRotation(currentPose.get(), targetPose).getZ()));
      m_alignThetaController.reset();
    })
    .unless(() -> m_lockState == DriveLockState.Locked)
    .until(() -> m_isAlignedToTarget)
    .withName("AlignDriveRotationToTarget");
  }

  public void setIdleMode(IdleMode idleMode) {
    m_idleMode = idleMode;
    for (int i = 0; i < m_swerveModules.length; i++) {
      m_swerveModules[i].setIdleMode(m_idleMode);
    }
  }

  private void updateTelemetry() {
    SmartDashboard.putString("Robot/Drive/LockState", m_lockState.toString());
    SmartDashboard.putString("Robot/Drive/IdleMode", m_idleMode.toString().substring(1));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder); 
    for (int i = 0; i < m_swerveModules.length; i++) {
      m_swerveModules[i].initSendable(builder);
    }
  }
}
