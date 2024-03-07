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
      new Pose2d(),
      Constants.Sensors.Pose.kStateStandardDeviations,
      Constants.Sensors.Pose.kVisionStandardDeviations);
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
    m_poseEstimator.update(m_gyroRotation.get(), m_swerveModulePositions.get());
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
    return Utils.getYawToPose(getPose(), getTargetPose().toPose2d());
  }

  public double getTargetPitch() {
    return Utils.getPitchToPose(new Pose3d(getPose()), getTargetPose());
  }

  public double getTargetDistance() {
    return Utils.getDistanceToPose(getPose(), getTargetPose().toPose2d());
  }

  private void updateTelemetry() {
    Pose2d robotPose = getPose();
    SmartDashboard.putNumberArray("Robot/Pose/Raw", new double[] { robotPose.getX(), robotPose.getY(), robotPose.getRotation().getRadians() });
    SmartDashboard.putString("Robot/Pose", Utils.objectToJson(robotPose));
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
