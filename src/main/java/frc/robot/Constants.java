package frc.robot;

import static java.util.Map.entry;

import java.util.Map;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
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
import frc.robot.lib.common.Enums.AutoPath;
import frc.robot.lib.common.Records.LauncherArmPosition;
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

    public static final double kMaxSpeedMetersPerSecond = 6.32;
    public static final double kMaxAngularSpeed = 4 * Math.PI;

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

    public static final com.pathplanner.lib.util.PIDConstants kPathFollowerTranslationPIDConstants = new com.pathplanner.lib.util.PIDConstants(5, 0, 0);
    public static final com.pathplanner.lib.util.PIDConstants kPathFollowerRotationPIDConstants = new com.pathplanner.lib.util.PIDConstants(5, 0, 0);
    public static final PathConstraints kPathFindingConstraints = new PathConstraints(6.0, 3.9, Units.degreesToRadians(540), Units.degreesToRadians(720));

    public static final class SwerveModule {
      public static final int kDrivingMotorPinionTeeth = 14;
      public static final double kFreeSpeedRpm = 6238.73054766;
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
      public static final double kDrivingMotorMaxReverseOutput = -1;
      public static final double kDrivingMotorMaxForwardOutput = 1;
      public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
      public static final PIDConstants kDrivingMotorPIDConstants = new PIDConstants(0.04, 0, 0, 1 / kDriveWheelFreeSpeedRps);

      public static final int kTurningMotorCurrentLimit = 20;
      public static final double kTurningMotorMaxReverseOutput = -1;
      public static final double kTurningMotorMaxForwardOutput = 1;
      public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;
      public static final PIDConstants kTurningMotorPIDConstants = new PIDConstants(1, 0, 0, 0);
    }
  }

  public static final class Intake {
    public static final int kTopBeltMotorCANId = 18;
    public static final int kBottomBeltMotorCANId = 19;
    public static final int kRollerMotorCANId = 20;

    public static final int kTopBeltMotorCurrentLimit = 60;
    public static final double kTopBeltMotorMaxReverseOutput = -0.6;
    public static final double kTopBeltMotorMaxForwardOutput = 0.6;
    public static final IdleMode kTopBeltMotorIdleMode = IdleMode.kBrake;

    public static final int kBottomBeltMotorCurrentLimit = 60;
    public static final double kBottomBeltMotorMaxReverseOutput = -0.6;
    public static final double kBottomBeltMotorMaxForwardOutput = 0.6;
    public static final IdleMode kBottomBeltMotorIdleMode = IdleMode.kCoast;

    public static final double kIntakeBeltSpeeds = 0.65;
    public static final double kIntakeBeltWaitTime = 0.033;

    public static final int kRollerMotorCurrentLimit = 60;
    public static final double kRollerMotorMaxReverseOutput = -0.6;
    public static final double kRollerMotorMaxForwardOutput = 0.6;
    public static final IdleMode kRollerMotorIdleMode = IdleMode.kBrake;
  }

  public static final class Launcher {
    public static final int kArmMotorCANId = 11;
    public static final int kTopRollerMotorCANId = 12;
    public static final int kBottomRollerMotorCANId = 13;

    public static final int kArmMotorCurrentLimit = 60;
    public static final double kArmMotorMaxReverseOutput = -1.0;
    public static final double kArmMotorMaxForwardOutput = 1.0;
    public static final IdleMode kArmMotorIdleMode = IdleMode.kBrake;
    public static final PIDConstants kArmMotorPIDConstants = new PIDConstants(0.0003, 0, 0.00015, 1 / 16.8);
    public static final double kArmMotorForwardSoftLimit = 14.5;
    public static final double kArmMotorReverseSoftLimit = 1;
    public static final double kArmMotorPositionConversionFactor = 1.0 / 3.0;
    public static final double kArmMotorVelocityConversionFactor = kArmMotorPositionConversionFactor / 60.0;
    public static final double kArmMotorSmartMotionMaxVelocity = (33.0 / kArmMotorPositionConversionFactor) * 60;
    public static final double kArmMotorSmartMotionMaxAccel = 100.0 / kArmMotorVelocityConversionFactor;

    public static final int kTopRollerMotorCurrentLimit = 100;
    public static final double kTopRollerMotorMaxReverseOutput = -1.0; 
    public static final double kTopRollerMotorMaxForwardOutput = 1.0;
    public static final IdleMode kTopRollerMotorIdleMode = IdleMode.kBrake;

    public static final int kBottomRollerMotorCurrentLimit = 100;
    public static final double kBottomRollerMotorMaxReverseOutput = -1.0;
    public static final double kBottomRollerMotorMaxForwardOutput = 1.0;
    public static final IdleMode kBottomRollerMotorIdleMode = IdleMode.kBrake;

    public static final LauncherRollerSpeeds kDefaultLauncherSpeeds = new LauncherRollerSpeeds(0.8, 0.8);
    public static final LauncherRollerSpeeds kWarmupLauncherSpeeds = new LauncherRollerSpeeds(0.6, 0.6);
    public static final LauncherRollerSpeeds kShuttleLauncherSpeeds = new LauncherRollerSpeeds(0.6, 0.6);
    public static final LauncherRollerSpeeds kAmpLauncherSpeeds = new LauncherRollerSpeeds(0.26, 0.26);
    public static final LauncherRollerSpeeds kTrapLauncherSpeeds = new LauncherRollerSpeeds(0.60, 0.61);

    public static final double kArmTargetAlignmentPositionTolerance = 0.1;
    
    public static final double kArmPositionIntake = 7.0;
    public static final double kArmPositionAmp = 13;
    public static final double kArmPositionShuttle = 12.0;

    public static final double kArmPositionSubwoofer = 12.9;
    public static final double kArmPositionPodium = 10.35; // TODO: Tune

    public static final LauncherArmPosition[] kArmPositions = new LauncherArmPosition[] {
      new LauncherArmPosition(1.00, 13),
      new LauncherArmPosition(1.35, 12.7),
      new LauncherArmPosition(2.3, 8.0),
      new LauncherArmPosition(3.65, 6.2),
      new LauncherArmPosition(5.0, 4.3),
      new LauncherArmPosition(6.2, 4.0)
    };
  }

  public static final class Climber {
    public static final int kArmMotorCANId = 16;
    public static final int kRollerMotorCANId = 17;

    public static final int kArmMotorCurrentLimit = 100;
    public static final double kArmMotorMaxReverseOutput = -1.0;
    public static final double kArmMotorMaxForwardOutput = 1.0;
    public static final IdleMode kArmMotorIdleMode = IdleMode.kBrake;
    public static final PIDConstants kArmMotorPIDConstants = new PIDConstants(0.05, 0, 0, 0);
    public static final double kArmMotorForwardSoftLimit = 30.0;
    public static final double kArmMotorReverseSoftLimit = 0.0; 
  }

  public static final class Sensors {
    public static final class Gyro {
      public static final IMUAxis kIMUAxisYaw = IMUAxis.kZ;
      public static final IMUAxis kIMUAxisRoll = IMUAxis.kY;
      public static final IMUAxis kIMUAxisPitch = IMUAxis.kX;
      public static final SPI.Port kSPIPort = SPI.Port.kOnboardCS0;
      public static final CalibrationTime kCalibrationTime = CalibrationTime._4s;
    }

    public static final class Pose {
      public static final Map<String, Transform3d> kPoseSensors = Map.ofEntries(
        entry(
          "Rear",
          new Transform3d(
            new Translation3d(Units.inchesToMeters(-3.25), Units.inchesToMeters(-10.75), Units.inchesToMeters(18.0)),
            new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(-24.0), Units.degreesToRadians(180))
          )
        ),
        entry(
          "Side",
          new Transform3d(
            new Translation3d(Units.inchesToMeters(-3.25), Units.inchesToMeters(-11.5), Units.inchesToMeters(15.5)),
            new Rotation3d(0, Units.degreesToRadians(-28.4), Units.degreesToRadians(-90))
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
      public static final class LauncherBottom {
        public static final String kSensorName = "LauncherBottom";
        public static final int kChannel = 7;
      }
      public static final class LauncherTop {
        public static final String kSensorName = "LauncherTop";
        public static final int kChannel = 5;
      }
      public static final class Climber {
        public static final String kSensorName = "Climber";
        public static final int kChannel = 3;
      }
    }

    public static final class Object {
      public static final String kCameraName = "Front";
      public static final String kObjectName = "Note";
      public static final double kObjectRangeYaw = 10.0;
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

        // Trap Positions (BLUE)
        // Right: 4.3, 3.0
        // Left: 4.2, 5.1
        // Middle: 6.1, 4.1
      }
    }

    public static final class Auto {
      public static final Map<AutoPath, PathPlannerPath> kPaths = Map.ofEntries(
        entry(AutoPath.ScorePreload1, PathPlannerPath.fromPathFile("ScorePreload1")),
        entry(AutoPath.ScorePreload2, PathPlannerPath.fromPathFile("ScorePreload2")),
        entry(AutoPath.ScorePreload3, PathPlannerPath.fromPathFile("ScorePreload3")),
        entry(AutoPath.Pickup1, PathPlannerPath.fromPathFile("Pickup1")),
        entry(AutoPath.Pickup2, PathPlannerPath.fromPathFile("Pickup2")),
        entry(AutoPath.Pickup3, PathPlannerPath.fromPathFile("Pickup3")),
        entry(AutoPath.Pickup4, PathPlannerPath.fromPathFile("Pickup4")),
        entry(AutoPath.Pickup5, PathPlannerPath.fromPathFile("Pickup5")),
        entry(AutoPath.Pickup61, PathPlannerPath.fromPathFile("Pickup61")),
        entry(AutoPath.Pickup62, PathPlannerPath.fromPathFile("Pickup62")),
        entry(AutoPath.Pickup63, PathPlannerPath.fromPathFile("Pickup63")),
        entry(AutoPath.Pickup72, PathPlannerPath.fromPathFile("Pickup72")),
        entry(AutoPath.Pickup73, PathPlannerPath.fromPathFile("Pickup73")),
        entry(AutoPath.Pickup8, PathPlannerPath.fromPathFile("Pickup8")),
        entry(AutoPath.ScoreStage1, PathPlannerPath.fromPathFile("ScoreStage1")),
        entry(AutoPath.ScoreStage2, PathPlannerPath.fromPathFile("ScoreStage2")),
        entry(AutoPath.ScoreStage3, PathPlannerPath.fromPathFile("ScoreStage3"))
      );
    }
  }
}
