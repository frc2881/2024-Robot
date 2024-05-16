package frc.robot.subsystems;

import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.common.Utils;
import frc.robot.lib.sensors.PoseSensor;

public class PoseSubsystem extends SubsystemBase {
  private final List<PoseSensor> m_poseSensors;
  private final Supplier<Rotation2d> m_gyroRotation;
  private final Supplier<SwerveModulePosition[]> m_swerveModulePositions;
  private final SwerveDrivePoseEstimator m_poseEstimator;

  public PoseSubsystem(
    List<PoseSensor> poseSensors,
    Supplier<Rotation2d> gyroRotation,
    Supplier<SwerveModulePosition[]> swerveModulePositions
  ) {
    m_poseSensors = poseSensors;
    m_gyroRotation = gyroRotation;
    m_swerveModulePositions = swerveModulePositions;
    m_poseEstimator = new SwerveDrivePoseEstimator(
      Constants.Drive.kSwerveDriveKinematics,
      m_gyroRotation.get(),
      m_swerveModulePositions.get(),
      new Pose2d()
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

  private void updatePose() {
    m_poseEstimator.update(m_gyroRotation.get(), m_swerveModulePositions.get());
    m_poseSensors.forEach(poseSensor -> {
      poseSensor.getEstimatedRobotPose().ifPresent(estimatedRobotPose -> {
        Pose2d pose = estimatedRobotPose.estimatedPose.toPose2d();
        if (isPoseOnField(pose)) {
          if (estimatedRobotPose.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
            m_poseEstimator.addVisionMeasurement(pose, estimatedRobotPose.timestampSeconds, Constants.Sensors.Pose.kVisionMultiTagStandardDeviations);
          } else {
            for (PhotonTrackedTarget target : estimatedRobotPose.targetsUsed) {
              if (Utils.isValueInRange(target.getPoseAmbiguity(), 0.0, Constants.Sensors.Pose.kVisionMaxPoseAmbiguity)) {
                m_poseEstimator.addVisionMeasurement(pose, estimatedRobotPose.timestampSeconds, Constants.Sensors.Pose.kVisionSingleTagStandardDeviations);
                break;
              }
            }
          }
        }
      });
    });
  }

  public void resetPose(Pose2d pose) {
    // NO-OP as current pose is always maintained by pose sensors in the configuration for this robot
    // m_poseEstimator.resetPosition(m_gyroRotation.get(), m_swerveModulePositions.get(), pose);
  }

  private boolean isPoseOnField(Pose2d pose) {
    double x = pose.getX();
    double y = pose.getY();
    return (x >= 0.0 && x <= Constants.Game.Field.kAprilTagFieldLayout.getFieldLength()) && (y >= 0.0 && y <= Constants.Game.Field.kAprilTagFieldLayout.getFieldWidth());
  }

  public boolean hasVisionTargets() {
    for (PoseSensor poseSensor : m_poseSensors) {
      if(poseSensor.hasTarget()){
        return true;
      }
    }
    return false;
  }

  public Pose3d getTargetPose() {
    return Utils.getValueForAlliance(
      Constants.Game.Field.Targets.kBlueSpeaker, 
      Constants.Game.Field.Targets.kRedSpeaker);
  }

  public double getTargetYaw() {
    Pose2d robotPose = getPose();
    Pose2d targetPose = getTargetPose().toPose2d().transformBy(
      new Transform2d(
        Utils.getValueForAlliance(
          Constants.Game.Field.Targets.kSpeakerTargetYawTransformX, 
          -Constants.Game.Field.Targets.kSpeakerTargetYawTransformX
        ), 
        Constants.Game.Field.Targets.kSpeakerTargetYTransform, 
        Rotation2d.fromDegrees(0.0)
      )
    );
    Translation2d targetTranslation = targetPose.relativeTo(robotPose).getTranslation();
    Rotation2d targetRotation = new Rotation2d(targetTranslation.getX(), targetTranslation.getY());
    targetRotation = targetRotation
      .rotateBy(Rotation2d.fromDegrees(180.0))
      .rotateBy(robotPose.getRotation());
    return Utils.wrapAngle(targetRotation.getDegrees());
  }

  public double getTargetPitch() {
    return Utils.getPitchToPose(
      new Pose3d(getPose()),
      getTargetPose().transformBy(
        new Transform3d(
          0.0, 0.0, 
          Constants.Game.Field.Targets.kSpeakerTargetPitchTransformZ,
          new Rotation3d()
        )
      )
    );
  }

  public double getTargetDistance() {
    return Utils.getDistanceToPose(
      getPose(),
      getTargetPose().toPose2d().transformBy(
        new Transform2d(
          Utils.getValueForAlliance(
            -Constants.Game.Field.Targets.kSpeakerTargetDistanceTransformX, 
            Constants.Game.Field.Targets.kSpeakerTargetDistanceTransformX
          ), 
          0.0, Rotation2d.fromDegrees(0.0)
        )
      )
    );
  }

  private void updateTelemetry() {
    Pose2d robotPose = getPose();
    SmartDashboard.putNumberArray("Robot/Pose/Current", new double[] { robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees() });
    SmartDashboard.putNumber("Robot/Pose/Target/Yaw", getTargetYaw());
    SmartDashboard.putNumber("Robot/Pose/Target/Pitch", getTargetPitch());
    SmartDashboard.putNumber("Robot/Pose/Target/Distance", getTargetDistance());
  }
}
