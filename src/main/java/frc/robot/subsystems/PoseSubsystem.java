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
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.Utils;
import frc.robot.lib.sensors.Camera;

public class PoseSubsystem extends SubsystemBase {
  private final SwerveDrivePoseEstimator m_poseEstimator;
  private final Supplier<Rotation2d> m_rotationSupplier;
  private final Supplier<SwerveModulePosition[]> m_swerveModulePositionSupplier;
  private final List<Camera> m_cameras;
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
      new Pose2d());

    m_cameras = new ArrayList<Camera>();
    Constants.Vision.kCameras.forEach((cameraName, cameraTransform) -> {
        m_cameras.add(new Camera(
          cameraName, cameraTransform, 
          Constants.Vision.kPoseStrategy, 
          Constants.Vision.kFallbackPoseStrategy, 
          Constants.Vision.kSingleTagStandardDeviations, 
          Constants.Vision.kMultiTagStandardDeviations, 
          Constants.Vision.kAprilTagFieldLayout)
        );
    });

    updateAprilTagFieldLayoutData();
  }

  @Override
  public void periodic() {
    updateAlliance();
    updatePose();
    updateTelemetry();
  }

  private void updateAprilTagFieldLayoutData() {
    try {
      Path filePath = Paths.get("april-tag-field-layout.json");
      Constants.Vision.kAprilTagFieldLayout.serialize(filePath);
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
      Constants.Vision.kAprilTagFieldLayout.setOrigin(
        m_alliance == Alliance.Red 
          ? OriginPosition.kRedAllianceWallRightSide
          : OriginPosition.kBlueAllianceWallRightSide
      );
    }
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void updatePose() {
    m_poseEstimator.update(m_rotationSupplier.get(), m_swerveModulePositionSupplier.get());
    m_cameras.forEach(camera -> {
      camera.getEstimatedGlobalPose().ifPresent(globalPose -> {
        Pose2d pose = globalPose.estimatedPose.toPose2d();
        m_poseEstimator.addVisionMeasurement(
          pose, 
          globalPose.timestampSeconds, 
          camera.getEstimatedStandardDeviations(pose)
        );
      });
    });
  }

  public void resetPose() {
    resetPose(new Pose2d());
  }

  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(m_rotationSupplier.get(), m_swerveModulePositionSupplier.get(), pose);
  }

  public void reset() {
    // TODO: reset the subsystem if needed
  }

  private void updateTelemetry() {
    SmartDashboard.putString("Robot/Pose/CurrentPose", Utils.objectToJson(getPose()));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // TODO: determine how pose needs to be sent to log for match replay
  }
}
