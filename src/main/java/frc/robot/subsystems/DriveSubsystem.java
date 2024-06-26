package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.common.Enums.DriveDriftCorrection;
import frc.robot.lib.common.Enums.DriveLockState;
import frc.robot.lib.common.Enums.DriveOrientation;
import frc.robot.lib.common.Enums.DriveSpeedMode;
import frc.robot.lib.common.Enums.SwerveModuleLocation;
import frc.robot.lib.drive.SwerveModule;

public class DriveSubsystem extends SubsystemBase {
  private Supplier<Double> m_gyroHeading;
  private final SwerveModule m_swerveModuleFrontLeft;
  private final SwerveModule m_swerveModuleFrontRight;
  private final SwerveModule m_swerveModuleRearLeft;
  private final SwerveModule m_swerveModuleRearRight;
  private final PIDController m_driftCorrectionThetaController;
  private final PIDController m_targetAlignmentThetaController;
  private final SlewRateLimiter m_driveInputXFilter;
  private final SlewRateLimiter m_driveInputYFilter;
  private final SlewRateLimiter m_driveInputRotFilter;

  private DriveOrientation m_orientation = DriveOrientation.Field;
  private DriveSpeedMode m_speedMode = DriveSpeedMode.Competition;
  private DriveLockState m_lockState = DriveLockState.Unlocked;
  private DriveDriftCorrection m_driftCorrection = DriveDriftCorrection.Enabled;
  private boolean m_isDriftCorrectionActive = false;
  private boolean m_isAlignedToTarget = false;

  public DriveSubsystem(Supplier<Double> gyroHeading) {
    m_gyroHeading = gyroHeading;

    m_swerveModuleFrontLeft = new SwerveModule(
      SwerveModuleLocation.FrontLeft,
      Constants.Drive.kSwerveModuleFrontLeftDrivingMotorCANId,
      Constants.Drive.kSwerveModuleFrontLeftTurningMotorCANId,
      Constants.Drive.kSwerveModuleFrontLeftOffset);

    m_swerveModuleFrontRight = new SwerveModule(
      SwerveModuleLocation.FrontRight,
      Constants.Drive.kSwerveModuleFrontRightDrivingMotorCANId,
      Constants.Drive.kSwerveModuleFrontRightTurningMotorCANId,
      Constants.Drive.kSwerveModuleFrontRightOffset);

    m_swerveModuleRearLeft = new SwerveModule(
      SwerveModuleLocation.RearLeft,
      Constants.Drive.kSwerveModuleRearLeftDrivingMotorCANId,
      Constants.Drive.kSwerveModuleRearLeftTurningMotorCANId,
      Constants.Drive.kSwerveModuleRearLeftOffset);

    m_swerveModuleRearRight = new SwerveModule(
      SwerveModuleLocation.RearRight,
      Constants.Drive.kSwerveModuleRearRightDrivingMotorCANId,
      Constants.Drive.kSwerveModuleRearRightTurningMotorCANId, 
      Constants.Drive.kSwerveModuleRearRightOffset);

    m_driftCorrectionThetaController = new PIDController(Constants.Drive.kDriftCorrectionThetaControllerPIDConstants.P(), Constants.Drive.kDriftCorrectionThetaControllerPIDConstants.I(), Constants.Drive.kDriftCorrectionThetaControllerPIDConstants.D());
    m_driftCorrectionThetaController.enableContinuousInput(-180.0, 180.0);
    m_driftCorrectionThetaController.setTolerance(Constants.Drive.kDriftCorrectionThetaControllerPositionTolerance, Constants.Drive.kDriftCorrectionThetaControllerVelocityTolerance);

    m_targetAlignmentThetaController = new PIDController(Constants.Drive.kTargetAlignmentThetaControllerPIDConstants.P(), Constants.Drive.kTargetAlignmentThetaControllerPIDConstants.I(), Constants.Drive.kTargetAlignmentThetaControllerPIDConstants.D());
    m_targetAlignmentThetaController.enableContinuousInput(-180.0, 180.0);
    m_targetAlignmentThetaController.setTolerance(Constants.Drive.kTargetAlignmentThetaControllerPositionTolerance, Constants.Drive.kTargetAlignmentThetaControllerVelocityTolerance);

    m_driveInputXFilter = new SlewRateLimiter(Constants.Drive.kDriveInputRateLimit);
    m_driveInputYFilter = new SlewRateLimiter(Constants.Drive.kDriveInputRateLimit);
    m_driveInputRotFilter = new SlewRateLimiter(Constants.Drive.kDriveInputRateLimit);

    SendableChooser<DriveSpeedMode> driveSpeedModeChooser = new SendableChooser<DriveSpeedMode>();
    driveSpeedModeChooser.setDefaultOption(DriveSpeedMode.Competition.toString(), DriveSpeedMode.Competition);
    driveSpeedModeChooser.addOption(DriveSpeedMode.Training.toString(), DriveSpeedMode.Training);
    driveSpeedModeChooser.onChange(speedMode -> setSpeedMode(speedMode));
    SmartDashboard.putData("Robot/Drive/SpeedMode", driveSpeedModeChooser);

    SendableChooser<DriveOrientation> driveOrientationChooser = new SendableChooser<DriveOrientation>();
    driveOrientationChooser.setDefaultOption(DriveOrientation.Field.toString(), DriveOrientation.Field);
    driveOrientationChooser.addOption(DriveOrientation.Robot.toString(), DriveOrientation.Robot);
    driveOrientationChooser.onChange(orientation -> setOrientation(orientation));
    SmartDashboard.putData("Robot/Drive/Orientation", driveOrientationChooser);

    SendableChooser<DriveDriftCorrection> driveDriftCorrectionChooser = new SendableChooser<DriveDriftCorrection>();
    driveDriftCorrectionChooser.setDefaultOption(DriveDriftCorrection.Enabled.toString(), DriveDriftCorrection.Enabled);
    driveDriftCorrectionChooser.addOption(DriveDriftCorrection.Disabled.toString(), DriveDriftCorrection.Disabled);
    driveDriftCorrectionChooser.onChange(driftCorrection -> setDriftCorrection(driftCorrection));
    SmartDashboard.putData("Robot/Drive/DriftCorrection", driveDriftCorrectionChooser);

    SendableChooser<IdleMode> idleModeChooser = new SendableChooser<IdleMode>();
    idleModeChooser.setDefaultOption("Brake", IdleMode.kBrake);
    idleModeChooser.addOption("Coast", IdleMode.kCoast);
    idleModeChooser.onChange(idleMode -> setIdleMode(idleMode));
    SmartDashboard.putData("Robot/Drive/IdleMode", idleModeChooser);
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  public Command driveWithControllerCommand(Supplier<Double> controllerLeftY, Supplier<Double> controllerLeftX, Supplier<Double> controllerRightX) {
    return 
    run(() -> {
      double speedX = controllerLeftY.get();
      double speedY = controllerLeftX.get();
      double speedRotation = controllerRightX.get();
      if (m_driftCorrection == DriveDriftCorrection.Enabled) {
        boolean isRotating = (speedRotation != 0.0);
        boolean isTranslating = ((speedX != 0.0) || (speedY != 0.0));
        if (!m_isDriftCorrectionActive && !isRotating && isTranslating) {
          m_isDriftCorrectionActive = true;
          m_driftCorrectionThetaController.reset();
          m_driftCorrectionThetaController.setSetpoint(m_gyroHeading.get());
        } else if (isRotating || !isTranslating) {
          m_isDriftCorrectionActive = false;
        }
        if (m_isDriftCorrectionActive) {
          speedRotation = m_driftCorrectionThetaController.calculate(m_gyroHeading.get());
          if (m_driftCorrectionThetaController.atSetpoint()) {
            speedRotation = 0.0;
          }
        }
      }
      if (m_speedMode == DriveSpeedMode.Training) {
        speedX = m_driveInputXFilter.calculate(speedX * Constants.Drive.kDriveInputLimiter);
        speedY = m_driveInputYFilter.calculate(speedY * Constants.Drive.kDriveInputLimiter);
        speedRotation = m_driveInputRotFilter.calculate(speedRotation * Constants.Drive.kDriveInputLimiter);
      }
      drive(speedX, speedY, speedRotation);
    })
    .onlyIf(() -> m_lockState != DriveLockState.Locked)
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
    return new SwerveModulePosition[] {
      m_swerveModuleFrontLeft.getPosition(),
      m_swerveModuleFrontRight.getPosition(),
      m_swerveModuleRearLeft.getPosition(),
      m_swerveModuleRearRight.getPosition()
    };
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
      m_swerveModuleFrontLeft.getState(),
      m_swerveModuleFrontRight.getState(),
      m_swerveModuleRearLeft.getState(),
      m_swerveModuleRearRight.getState()
    };
  }

  private void setSwerveModuleStates(SwerveModuleState[] swerveModuleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Drive.kMaxSpeedMetersPerSecond);
    m_swerveModuleFrontLeft.setTargetState(swerveModuleStates[0]);
    m_swerveModuleFrontRight.setTargetState(swerveModuleStates[1]);
    m_swerveModuleRearLeft.setTargetState(swerveModuleStates[2]);
    m_swerveModuleRearRight.setTargetState(swerveModuleStates[3]);
  }

  public void setLockState(DriveLockState lockState) {
    m_lockState = lockState;
    if (m_lockState == DriveLockState.Locked) {
      m_swerveModuleFrontLeft.setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
      m_swerveModuleFrontRight.setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
      m_swerveModuleRearLeft.setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
      m_swerveModuleRearRight.setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
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

  public Command alignToTargetCommand(Supplier<Pose2d> robotPose, Supplier<Double> targetYaw) {
    return
    run(() -> {
      double robotYaw = robotPose.get().getRotation().getDegrees();
      double speedRotation = m_targetAlignmentThetaController.calculate(robotYaw);
      speedRotation += Math.copySign(0.15, speedRotation);
      if (m_targetAlignmentThetaController.atSetpoint()) {
        speedRotation = 0.0;
        m_isAlignedToTarget = true;
      }
      setSwerveModuleStates(Constants.Drive.kSwerveDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, speedRotation)));
    })
    .beforeStarting(() -> {
      m_isAlignedToTarget = false;
      m_targetAlignmentThetaController.setSetpoint(targetYaw.get());
      m_targetAlignmentThetaController.reset();
    })
    .onlyIf(() -> m_lockState != DriveLockState.Locked)
    .until(() -> m_isAlignedToTarget)
    .withName("AlignDriveRotationToTarget");
  }

  public boolean isAlignedToTarget() {
    return m_isAlignedToTarget;
  }

  public void clearTargetAlignment() {
    m_isAlignedToTarget = false;
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

  public void setIdleMode(IdleMode idleMode) {
    m_swerveModuleFrontLeft.setIdleMode(idleMode);
    m_swerveModuleFrontRight.setIdleMode(idleMode);
    m_swerveModuleRearLeft.setIdleMode(idleMode);
    m_swerveModuleRearRight.setIdleMode(idleMode);
    SmartDashboard.putString("Robot/Drive/IdleMode/selected", idleMode == IdleMode.kBrake ? "Brake" : "Coast");
  }

  public void reset() {
    setIdleMode(IdleMode.kBrake);
    drive(0.0, 0.0, 0.0);
  }

  private void updateTelemetry() {
    m_swerveModuleFrontLeft.updateTelemetry();
    m_swerveModuleFrontRight.updateTelemetry();
    m_swerveModuleRearLeft.updateTelemetry();
    m_swerveModuleRearRight.updateTelemetry();
    SmartDashboard.putString("Robot/Drive/LockState", m_lockState.toString());
    SmartDashboard.putBoolean("Robot/Drive/IsAlignedToTarget", m_isAlignedToTarget);
  }
}
