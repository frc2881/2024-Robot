package frc.robot.subsystems;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
    m_poseEstimator.resetPosition(m_gyroRotation.get(), m_swerveModulePosition.get(), pose);
  }

  private void updateTelemetry() {
    SmartDashboard.putString("Robot/Pose", Utils.objectToJson(getPose()));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty("Pose", () -> Utils.objectToJson(getPose()), null);
  }
}
