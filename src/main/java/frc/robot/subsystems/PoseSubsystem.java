package frc.robot.subsystems;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.Utils;
import frc.robot.lib.sensors.ObjectSensor;
import frc.robot.lib.sensors.PoseSensor;

public class PoseSubsystem extends SubsystemBase {
  private final SwerveDrivePoseEstimator m_poseEstimator;
  private final Supplier<Rotation2d> m_rotationSupplier;
  private final Supplier<SwerveModulePosition[]> m_swerveModulePositionSupplier;
  private final List<PoseSensor> m_poseSensors;
  private final ObjectSensor m_objectSensor;
  private Alliance m_alliance;

  public PoseSubsystem(
    Supplier<Rotation2d> rotationSupplier, 
    Supplier<SwerveModulePosition[]> swerveModulePositionSupplier
  ) {
    m_rotationSupplier = rotationSupplier;
    m_swerveModulePositionSupplier = swerveModulePositionSupplier;

    m_poseEstimator = new SwerveDrivePoseEstimator(
      Constants.Drive.kSwerveDriveKinematics, 
      m_rotationSupplier.get(), 
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

    m_objectSensor = new ObjectSensor(Constants.Sensors.Object.kCameraName);

    getAprilTagFieldLayoutData();
  }

  @Override
  public void periodic() {
    updateAlliance();
    updatePose();
    updateTelemetry();
  }

  private void getAprilTagFieldLayoutData() {
    try {
      Path filePath = Paths.get(Filesystem.getOperatingDirectory().getPath() + "/april-tag-field-layout.json");
      Constants.Game.Field.kAprilTagFieldLayout.serialize(filePath);
      SmartDashboard.putString("Robot/Pose/AprilTagFieldLayout", new String(Files.readAllBytes(filePath), StandardCharsets.UTF_8));
      Files.delete(filePath);
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  private void updateAlliance() {
    Alliance alliance = Robot.getAlliance();
    if (m_alliance != alliance) {
      m_alliance = alliance;
      Constants.Game.Field.kAprilTagFieldLayout.setOrigin(
        m_alliance == Alliance.Red 
          ? OriginPosition.kRedAllianceWallRightSide
          : OriginPosition.kBlueAllianceWallRightSide
      );
      // TODO: validate that cameras pickup origin position change automatically or need to be reset
    }
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void updatePose() {
    m_poseEstimator.update(m_rotationSupplier.get(), m_swerveModulePositionSupplier.get());
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
    m_poseEstimator.resetPosition(m_rotationSupplier.get(), m_swerveModulePositionSupplier.get(), pose);
  }

  private void updateTelemetry() {
    SmartDashboard.putString("Robot/Pose/CurrentPose", Utils.objectToJson(getPose()));
    m_objectSensor.getTrackedObject().ifPresent(trackedObject -> {
      SmartDashboard.putString("Robot/Pose/TrackedObject", Utils.objectToJson(trackedObject));
    });
    
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty("CurrentPose", () -> Utils.objectToJson(getPose()), null);
    // TODO: determine how pose needs to be sent to log for match replay
  }
}
