package frc.robot.subsystems;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.common.Utils;
import frc.robot.lib.sensors.PoseSensor;

public class PoseSubsystem extends SubsystemBase {
  private final List<PoseSensor> m_poseSensors;
  private final Supplier<Rotation2d> m_gyroRotation;
  private final Supplier<SwerveModulePosition[]> m_swerveModulePosition;
  private final SwerveDrivePoseEstimator m_poseEstimator;

  public PoseSubsystem(
    List<PoseSensor> poseSensors,
    Supplier<Rotation2d> gyroRotation,
    Supplier<SwerveModulePosition[]> swerveModulePosition
  ) {
    m_poseSensors = poseSensors;
    m_gyroRotation = gyroRotation;
    m_swerveModulePosition = swerveModulePosition;
    m_poseEstimator = new SwerveDrivePoseEstimator(
      Constants.Drive.kSwerveDriveKinematics, 
      m_gyroRotation.get(),
      m_swerveModulePosition.get(), 
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
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
    m_poseEstimator.update(m_gyroRotation.get(), m_swerveModulePosition.get());
    m_poseSensors.forEach(poseSensor -> {
      poseSensor.getEstimatedGlobalPose().ifPresent(globalPose -> {
        Pose2d pose = globalPose.estimatedPose.toPose2d();
        m_poseEstimator.addVisionMeasurement(
          pose, 
          globalPose.timestampSeconds, 
          poseSensor.getEstimatedStandardDeviations(pose)
        );
      });
    });
  }

  public void resetPose(Pose2d pose) {
    // NO-OP as current pose is always maintained by pose sensors in the configuration for this robot
    // m_poseEstimator.resetPosition(m_gyroRotation.get(), m_swerveModulePosition.get(), pose);
  }

  public Pose3d getTargetPose() {
    return 
    Robot.getAlliance() == Alliance.Blue 
    ? Constants.Game.Field.Targets.kBlueSpeaker 
    : Constants.Game.Field.Targets.kRedSpeaker;
  }

  public double getTargetYaw() {
    double targetYaw = Math.toDegrees(Utils.getTargetRotation(getPose(), getTargetPose()).getZ());
    targetYaw -= Math.copySign(Constants.Sensors.Pose.kTargetAlignmentYawCorrection, targetYaw);
    if (Robot.getAlliance() == Alliance.Red) { targetYaw += 180; }
    return targetYaw;
  }

  public double getTargetPitch() {
    return Math.toDegrees(Utils.getTargetRotation(getPose(), getTargetPose()).getY());
  }

  public double getTargetDistance() {
    return getPose().getTranslation().getDistance(getTargetPose().toPose2d().getTranslation()); 
  }

  private void updateTelemetry() {
    SmartDashboard.putString("Robot/Pose", Utils.objectToJson(getPose()));
    SmartDashboard.putString("Robot/Pose/Target/Pose", Utils.objectToJson(getTargetPose()));
    SmartDashboard.putNumber("Robot/Pose/Target/Yaw", getTargetYaw());
    SmartDashboard.putNumber("Robot/Pose/Target/Pitch", getTargetPitch());
    SmartDashboard.putNumber("Robot/Pose/Target/Distance", getTargetDistance());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty("Pose", () -> Utils.objectToJson(getPose()), null);
  }
}
