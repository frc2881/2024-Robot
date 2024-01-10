// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.Utils;
import frc.robot.lib.sensors.Camera;
import edu.wpi.first.math.Matrix;


public class PoseSubsystem extends SubsystemBase {
  private final SwerveDrivePoseEstimator m_poseEstimator;
  private final Supplier<Rotation2d> m_rotationSupplier;
  private final Supplier<SwerveModulePosition[]> m_swerveModulePositionSupplier;
  private final List<Camera> m_cameras;

  /** Creates a new Pose. */
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
          Constants.Vision.kTagLayout)
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
    m_poseEstimator.update(m_rotationSupplier.get(), m_swerveModulePositionSupplier.get());
    m_cameras.forEach(camera -> {
      camera.getEstimatedGlobalPose().ifPresent(globalPose -> {
        Pose2d pose = globalPose.estimatedPose.toPose2d();
        m_poseEstimator.addVisionMeasurement(
          pose, 
          globalPose.timestampSeconds, 
          camera.getEstimatedStandardDeviations(pose));
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
    // TODO: reset the subsystem
  }

  private void updateTelemetry() {
    SmartDashboard.putString("Robot/Pose", Utils.objectToJson(getPose()));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  // TODO: Determine how pose needs to be sent to log for match replay
  }
}
