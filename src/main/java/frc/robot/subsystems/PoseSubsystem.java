package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.Utils;
import frc.robot.lib.sensors.GyroSensor;
import frc.robot.lib.sensors.PoseSensor;

public class PoseSubsystem extends SubsystemBase {
  private final SwerveDrivePoseEstimator m_poseEstimator;
  private final GyroSensor m_gyroSensor;
  private final Supplier<SwerveModulePosition[]> m_swerveModulePositionSupplier;
  private final List<PoseSensor> m_poseSensors;

  public PoseSubsystem(
    GyroSensor gyroSensor, 
    Supplier<SwerveModulePosition[]> swerveModulePositionSupplier
  ) {
    m_gyroSensor = gyroSensor;
    m_swerveModulePositionSupplier = swerveModulePositionSupplier;

    m_poseEstimator = new SwerveDrivePoseEstimator(
      Constants.Drive.kSwerveDriveKinematics, 
      m_gyroSensor.getRotation2d(), 
      m_swerveModulePositionSupplier.get(), 
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

    m_poseSensors = new ArrayList<PoseSensor>();
    Constants.Sensors.Pose.kPoseSensors.forEach((cameraName, cameraTransform) -> {
        m_poseSensors.add(new PoseSensor(
          cameraName, cameraTransform, 
          Constants.Sensors.Pose.kPoseStrategy, 
          Constants.Sensors.Pose.kFallbackPoseStrategy, 
          Constants.Sensors.Pose.kSingleTagStandardDeviations, 
          Constants.Sensors.Pose.kMultiTagStandardDeviations, 
          Constants.Game.Field.kAprilTagFieldLayout)
        );
    });
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
    m_poseEstimator.update(m_gyroSensor.getRotation2d(), m_swerveModulePositionSupplier.get());
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

  public Command resetPoseCommand() {
    return Commands.runOnce(
      () -> resetPose())
      .ignoringDisable(true)
      .withName("ResetPose");
  }

  public void resetPose() {
    resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
  }

  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(m_gyroSensor.getRotation2d(), m_swerveModulePositionSupplier.get(), pose);
  }

  private void updateTelemetry() {
    SmartDashboard.putString("Robot/Pose", Utils.objectToJson(new Pose3d(getPose()).rotateBy(m_gyroSensor.getRotation3d())));
    m_poseSensors.forEach(poseSensor -> poseSensor.updateTelemetry());    
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty("Pose", () -> Utils.objectToJson(getPose()), null);
    // TODO: determine how pose needs to be sent to log for match replay
  }
}
