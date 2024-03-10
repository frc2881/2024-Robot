package frc.robot;

import static java.util.Map.entry;

import java.util.Map;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.lib.common.Records.AutoPoses;
import frc.robot.lib.common.Records.LauncherArmPosition;
import frc.robot.lib.common.Records.LauncherRollerSpeedForPosition;
import frc.robot.lib.common.Records.LauncherRollerSpeeds;
import frc.robot.lib.common.Records.PIDConstants;

public final class Constants {

  public static final class Controllers {
    public static final int kDriverControllerPort = 0; 
    public static final int kOperatorControllerPort = 1; 
    public static final double kInputDeadband = 0.1; 
  }

  public static final class Drive {
    public static final int kSwerveModuleFrontLeftDrivingMotorCANId = 3;
    public static final int kSwerveModuleFrontLeftTurningMotorCANId = 4;
    public static final int kSwerveModuleFrontRightDrivingMotorCANId = 7;
    public static final int kSwerveModuleFrontRightTurningMotorCANId = 8;
    public static final int kSwerveModuleRearLeftDrivingMotorCANId = 5;
    public static final int kSwerveModuleRearLeftTurningMotorCANId = 6;
    public static final int kSwerveModuleRearRightDrivingMotorCANId = 9;
    public static final int kSwerveModuleRearRightTurningMotorCANId = 10;

    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    public static final double kWheelBase = Units.inchesToMeters(21.5);
    public static final double kDriveBaseRadius = new Translation2d().getDistance(new Translation2d(kWheelBase / 2, kTrackWidth / 2));

    public static final double kMaxSpeedMetersPerSecond = 5.28;
    public static final double kMaxAngularSpeed = 3 * Math.PI;

    public static final double kSwerveModuleFrontLeftOffset = -Math.PI / 2;
    public static final double kSwerveModuleFrontRightOffset = 0;
    public static final double kSwerveModuleRearLeftOffset = Math.PI;
    public static final double kSwerveModuleRearRightOffset = Math.PI / 2;

    public static final Translation2d kSwerveModuleFrontLeftTranslation = new Translation2d(Constants.Drive.kWheelBase / 2, Constants.Drive.kTrackWidth / 2);
    public static final Translation2d kSwerveModuleFrontRightTranslation = new Translation2d(Constants.Drive.kWheelBase / 2, -Constants.Drive.kTrackWidth / 2);
    public static final Translation2d kSwerveModuleRearLeftTranslation = new Translation2d(-Constants.Drive.kWheelBase / 2, Constants.Drive.kTrackWidth / 2);
    public static final Translation2d kSwerveModuleRearRightTranslation = new Translation2d(-Constants.Drive.kWheelBase / 2, -Constants.Drive.kTrackWidth / 2);

    public static final SwerveDriveKinematics kSwerveDriveKinematics = new SwerveDriveKinematics(
      kSwerveModuleFrontLeftTranslation, 
      kSwerveModuleFrontRightTranslation, 
      kSwerveModuleRearLeftTranslation, 
      kSwerveModuleRearRightTranslation
    );

    public static final PIDConstants kDriftCorrectionThetaControllerPIDConstants = new PIDConstants(0.01, 0, 0, 0);
    public static final double kDriftCorrectionThetaControllerPositionTolerance = 0.5;
    public static final double kDriftCorrectionThetaControllerVelocityTolerance = 0.5;

    public static final PIDConstants kTargetAlignmentThetaControllerPIDConstants = new PIDConstants(0.1, 0, 0.01, 0);
    public static final double kTargetAlignmentThetaControllerPositionTolerance = 0.5;
    public static final double kTargetAlignmentThetaControllerVelocityTolerance = 0.5;

    public static final double kDriveInputLimiter = 0.6;
    public static final double kDriveInputRateLimit = 0.5;

    public static final com.pathplanner.lib.util.PIDConstants kPathFollowerTranslationPIDConstants = new com.pathplanner.lib.util.PIDConstants(0.5, 0, 0);
    public static final com.pathplanner.lib.util.PIDConstants kPathFollowerRotationPIDConstants = new com.pathplanner.lib.util.PIDConstants(0.7, 0, 0);
    public static final PathConstraints kPathFindingConstraints = new PathConstraints(1.0, 1.0, 540.00, 720.00);

    public static final class SwerveModule {
      public static final int kDrivingMotorPinionTeeth = 14;
      public static final double kFreeSpeedRpm = 5676;
      public static final double kWheelDiameterMeters = Units.inchesToMeters(3.0);
      public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
      public static final double kDrivingMotorReduction = (45.0 * 20) / (kDrivingMotorPinionTeeth * 15);
      public static final double kDrivingMotorFreeSpeedRps = kFreeSpeedRpm / 60;
      public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction; 
      public static final double kDrivingEncoderPositionConversionFactor = (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction;
      public static final double kDrivingEncoderVelocityConversionFactor = ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0;
      
      public static final boolean kTurningEncoderInverted = true;
      public static final double kTurningEncoderPositionConversionFactor = (2 * Math.PI);
      public static final double kTurningEncoderVelocityConversionFactor = (2 * Math.PI) / 60.0;
      public static final double kTurningEncoderPositionPIDMinInput = 0;
      public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionConversionFactor;
      
      public static final int kDrivingMotorCurrentLimit = 80;
      public static final double kDrivingMotorMinOutput = -1;
      public static final double kDrivingMotorMaxOutput = 1;
      public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
      public static final PIDConstants kDrivingMotorPIDConstants = new PIDConstants(0.04, 0, 0, 1 / kDriveWheelFreeSpeedRps);

      public static final int kTurningMotorCurrentLimit = 20;
      public static final double kTurningMotorMinOutput = -1;
      public static final double kTurningMotorMaxOutput = 1;
      public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;
      public static final PIDConstants kTurningMotorPIDConstants = new PIDConstants(1, 0, 0, 0);
    }
  }

  public static final class Intake {
    public static final int kTopBeltMotorCANId = 18;
    public static final int kBottomBeltMotorCANId = 19;
    public static final int kRollerMotorCANId = 20;

    public static final int kTopBeltMotorCurrentLimit = 60;
    public static final double kTopBeltMotorMinOutput = -0.6;
    public static final double kTopBeltMotorMaxOutput = 0.6;
    public static final IdleMode kTopBeltMotorIdleMode = IdleMode.kBrake;

    public static final int kBottomBeltMotorCurrentLimit = 60;
    public static final double kBottomBeltMotorMinOutput = -0.6;
    public static final double kBottomBeltMotorMaxOutput = 0.6;
    public static final IdleMode kBottomBeltMotorIdleMode = IdleMode.kCoast;

    public static final int kRollerMotorCurrentLimit = 60;
    public static final double kRollerMotorMinOutput = -0.6;
    public static final double kRollerMotorMaxOutput = 0.6;
    public static final IdleMode kRollerMotorIdleMode = IdleMode.kBrake;
  }

  public static final class Launcher {
    public static final int kArmMotorCANId = 11;
    public static final int kTopRollerMotorCANId = 12;
    public static final int kBottomRollerMotorCANId = 13;

    public static final int kArmMotorCurrentLimit = 60;
    public static final double kArmMotorMinOutput = -1.0;
    public static final double kArmMotorMaxOutput = 1.0;
    public static final IdleMode kArmMotorIdleMode = IdleMode.kBrake;
    public static final PIDConstants kArmMotorPIDConstants = new PIDConstants(0.0003, 0, 0.00015, 1 / 16.8);
    public static final double kArmMotorForwardSoftLimit = 14.5;
    public static final double kArmMotorReverseSoftLimit = 1;
    public static final double kArmMotorPositionConversionFactor = 1.0 / 3.0;
    public static final double kArmMotorVelocityConversionFactor = kArmMotorPositionConversionFactor / 60.0;
    public static final double kArmMotorSmartMotionMaxVelocity = (33.0 / kArmMotorPositionConversionFactor) * 60;
    public static final double kArmMotorSmartMotionMaxAccel = 100.0 / kArmMotorVelocityConversionFactor;

    public static final int kTopRollerMotorCurrentLimit = 100;
    public static final double kTopRollerMotorMinOutput = -1.0; 
    public static final double kTopRollerMotorMaxOutput = 1.0;
    public static final IdleMode kTopRollerMotorIdleMode = IdleMode.kBrake;

    public static final int kBottomRollerMotorCurrentLimit = 100;
    public static final double kBottomRollerMotorMinOutput = -1.0;
    public static final double kBottomRollerMotorMaxOutput = 1.0;
    public static final IdleMode kBottomRollerMotorIdleMode = IdleMode.kBrake;

    public static final LauncherRollerSpeeds kWarmupLauncherSpeeds = new LauncherRollerSpeeds(0.60, 0.60);
    public static final LauncherRollerSpeeds kAmpLauncherSpeeds = new LauncherRollerSpeeds(0.35, 0.35);
    
    public static final double kArmPositionIntake = 7.0;
    public static final double kArmPositionAmp = 11.4;
    public static final double kArmPositionSubwoofer = 12.9; // 1.35m
    public static final double kArmPositionShortRange = 10.35; // 1.84m
    public static final double kArmPositionMidRange = 7.3; // 2.78m
    public static final double kArmPositionLongRange = 4.4; // 5.37m
    public static final double kArmPositionShuttle = 12.0; // 1.35m

    public static final LauncherArmPosition[] kArmPositions = new LauncherArmPosition[] {
      new LauncherArmPosition(1.00, 13),
      new LauncherArmPosition(1.35, 12.7),
      new LauncherArmPosition(2.3, 8.55),
      new LauncherArmPosition(3.65, 6.25),
      new LauncherArmPosition(5.0, 4.5),
      new LauncherArmPosition(6.2, 4.1)
    };

    public static final LauncherRollerSpeedForPosition[] kRollerSpeeds = new LauncherRollerSpeedForPosition[] {
      new LauncherRollerSpeedForPosition(1.00, 0.5),
      new LauncherRollerSpeedForPosition(1.35, 0.55),
      new LauncherRollerSpeedForPosition(3.65, 0.75),
      new LauncherRollerSpeedForPosition(6.2, 0.8)
    };
  }

  public static final class Climber {
    public static final int kArmMotorCANId = 16;
    public static final int kRollerMotorCANId = 17;

    public static final int kArmMotorCurrentLimit = 60;
    public static final double kArmMotorMinOutput = -1.0;
    public static final double kArmMotorMaxOutput = 1.0;
    public static final IdleMode kArmMotorIdleMode = IdleMode.kBrake;
    public static final PIDConstants kArmMotorPIDConstants = new PIDConstants(0.0003, 0, 0.00015, 1 / 16.8);
    public static final double kArmMotorForwardSoftLimit = 38.7;
    public static final double kArmMotorReverseSoftLimit = 0.0; 
    public static final double kArmMotorPositionConversionFactor = 1.0 / 9.0; 
    public static final double kArmMotorVelocityConversionFactor = kArmMotorPositionConversionFactor / 60.0;
    public static final double kArmMotorSmartMotionMaxVelocity = (33.0 / kArmMotorPositionConversionFactor) * 60;
    public static final double kArmMotorSmartMotionMaxAccel = 100.0 / kArmMotorVelocityConversionFactor;

    public static final int kRollerMotorCurrentLimit = 60;
    public static final double kRollerMotorMinOutput = -1.0;
    public static final double kRollerMotorMaxOutput = 1.0;
    public static final IdleMode kRollerMotorIdleMode = IdleMode.kBrake;

    //37.7 is position for arm to lock
  }

  public static final class Sensors {
    public static final class Gyro {
      public static final IMUAxis kIMUAxisYaw = IMUAxis.kZ;
      public static final IMUAxis kIMUAxisRoll = IMUAxis.kY;
      public static final IMUAxis kIMUAxisPitch = IMUAxis.kX;
      public static final SPI.Port kSPIPort = SPI.Port.kOnboardCS0;
      public static final CalibrationTime kCalibrationTime = CalibrationTime._8s;
    }

    public static final class Pose {
      public static final Map<String, Transform3d> kPoseSensors = Map.ofEntries(
        entry(
          "Rear",
          new Transform3d(
            new Translation3d(Units.inchesToMeters(-5.5), Units.inchesToMeters(-11), Units.inchesToMeters(15.00)),
            new Rotation3d(Units.degreesToRadians(4.0), Units.degreesToRadians(-24.7), Units.degreesToRadians(183))
          )
        ),
        entry(
          "Side",
          new Transform3d(
            new Translation3d(Units.inchesToMeters(-5), Units.inchesToMeters(-12), Units.inchesToMeters(17.5)),
            new Rotation3d(0, Units.degreesToRadians(-23.8), Units.degreesToRadians(-87))
          )
        )
      );
      public static final PoseStrategy kPoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
      public static final PoseStrategy kFallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY;
      public static final double kMaxTargetPoseAmbiguity = 0.2;
      public static final double kMaxTargetsAverageDistance = 4.0;
      public static final Matrix<N3, N1> kStateStandardDeviations = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
      public static final Matrix<N3, N1> kVisionStandardDeviations = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10));
      public static final Matrix<N3, N1> kSingleTagStandardDeviations = VecBuilder.fill(1, 1, 2);
      public static final Matrix<N3, N1> kMultiTagStandardDeviations = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static final class BeamBreak {
      public static final class Intake {
        public static final String kSensorName = "Intake";
        public static final int kChannel = 9;
      }
      public static final class LauncherBottom {
        public static final String kSensorName = "LauncherBottom";
        public static final int kChannel = 7;
      }
      public static final class LauncherTop {
        public static final String kSensorName = "LauncherTop";
        public static final int kChannel = 5;
      }
    }

    public static final class Object {
      public static final String kCameraName = "Front";
      public static final String kObjectName = "Note";
      public static final double kObjectRangeYaw = 10.0;
    }

    public static final class Distance {
      public static final class Intake {
        public static final String kSensorName = "Intake";
        public static final double kMinTargetDistance = 0;
        public static final double kMaxTargetDistance = 250;
      }
      public static final class Launcher {
        public static final String kSensorName = "Launcher";
        public static final double kMinTargetDistance = 0;
        public static final double kMaxTargetDistance = 250;
      }
    }
  }

  public static final class Game {
    public static final class Field {
      public static final AprilTagFieldLayout kAprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

      public static final class Targets {
        public static final Pose3d kBlueSpeaker = kAprilTagFieldLayout.getTagPose(7).orElse(new Pose3d());
        public static final Pose3d kRedSpeaker = kAprilTagFieldLayout.getTagPose(4).orElse(new Pose3d());
        public static final Pose3d kBlueAmp = kAprilTagFieldLayout.getTagPose(5).orElse(new Pose3d());
        public static final Pose3d kRedAmp = kAprilTagFieldLayout.getTagPose(6).orElse(new Pose3d());

        public static final double kSpeakerTargetYawTransformX = Units.inchesToMeters(6);
        public static final double kSpeakerTargetPitchTransformZ = Units.inchesToMeters(24);
        public static final double kSpeakerTargetDistanceTransformX = Units.inchesToMeters(0);
      }

      public static final class AutoWaypoints {
        public static final AutoPoses kNotePreload1Poses = new AutoPoses(new Pose2d(), new Pose2d(1.84, 6.70, Rotation2d.fromDegrees(0)));
        public static final AutoPoses kScoreNotePreload2 = new AutoPoses(new Pose2d(), new Pose2d(1.84, 5.38, Rotation2d.fromDegrees(0)));
        public static final AutoPoses kScoreNotePreload3 = new AutoPoses(new Pose2d(), new Pose2d(1.84, 4.00, Rotation2d.fromDegrees(0)));

        public static final AutoPoses kNote1Poses = new AutoPoses(
          new Pose2d(2.78, 6.90, Rotation2d.fromDegrees(0)), 
          new Pose2d(2.78, 6.90, Rotation2d.fromDegrees(0)));

        public static final AutoPoses kNote2Poses = new AutoPoses(
          new Pose2d(2.78, 5.38, Rotation2d.fromDegrees(0)), 
          new Pose2d(2.78, 5.38, Rotation2d.fromDegrees(0)));

        public static final AutoPoses kNote3Poses = new AutoPoses(
          new Pose2d(2.52, 4.05, Rotation2d.fromDegrees(0)), 
          new Pose2d(2.52, 4.05, Rotation2d.fromDegrees(0)));

        public static final AutoPoses kNote4Poses = new AutoPoses(
          new Pose2d(7.87, 7.03, Rotation2d.fromDegrees(0)), 
          new Pose2d(5.37, 6.15, Rotation2d.fromDegrees(0)));

        public static final AutoPoses kNote5Poses = new AutoPoses(
          new Pose2d(7.87, 5.37, Rotation2d.fromDegrees(0)), 
          new Pose2d(5.37, 6.15, Rotation2d.fromDegrees(0)));

        public static final AutoPoses kNote6Poses = new AutoPoses(
          new Pose2d(7.87, 3.70, Rotation2d.fromDegrees(0)), 
          new Pose2d(5.37, 6.15, Rotation2d.fromDegrees(0)));

        public static final AutoPoses kNote7Poses = new AutoPoses(
          new Pose2d(7.87, 2.03, Rotation2d.fromDegrees(0)), 
          new Pose2d(5.37, 1.90, Rotation2d.fromDegrees(0)));

        public static final AutoPoses kNote8Poses = new AutoPoses(
          new Pose2d(7.87, 7.03, Rotation2d.fromDegrees(0)), 
          new Pose2d(5.37, 1.90, Rotation2d.fromDegrees(0)));
      }
    }
  }
}
