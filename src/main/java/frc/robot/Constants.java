package frc.robot;

import static java.util.Map.entry;

import java.util.Map;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
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
import frc.robot.commands.AutoCommands.NotePoses;
import frc.robot.lib.common.PIDConstants;
import frc.robot.subsystems.LauncherRollerSubsystem.RollerSpeeds;

public final class Constants {

  public static final class Controllers {
    public static final int kDriverControllerPort = 0; 
    public static final int kOperatorControllerPort = 1; 
    public static final double kInputDeadband = 0.1; 
  }

  public static final class Drive {
    public static final int kFrontLeftDrivingMotorCANId = 3;
    public static final int kFrontLeftTurningMotorCANId = 4;
    public static final int kRearLeftDrivingMotorCANId = 5;
    public static final int kRearLeftTurningMotorCANId = 6;
    public static final int kFrontRightDrivingMotorCANId = 7;
    public static final int kFrontRightTurningMotorCANId = 8;
    public static final int kRearRightDrivingMotorCANId = 9;
    public static final int kRearRightTurningMotorCANId = 10;

    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    public static final double kWheelBase = Units.inchesToMeters(21.5);
    public static final double kDriveBaseRadius = new Translation2d().getDistance(new Translation2d(kWheelBase / 2, kTrackWidth / 2));

    public static final double kMaxSpeedMetersPerSecond = 5.7424;
    public static final double kMaxAngularSpeed = 2 * Math.PI;

    public static final SwerveDriveKinematics kSwerveDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.Drive.kWheelBase / 2, Constants.Drive.kTrackWidth / 2),
      new Translation2d(Constants.Drive.kWheelBase / 2, -Constants.Drive.kTrackWidth / 2),
      new Translation2d(-Constants.Drive.kWheelBase / 2, Constants.Drive.kTrackWidth / 2),
      new Translation2d(-Constants.Drive.kWheelBase / 2, -Constants.Drive.kTrackWidth / 2)
    );

    public static final PIDConstants kDriftCorrectionThetaControllerPIDConstants = new PIDConstants(0.01, 0, 0);
    public static final double kDriftCorrectionThetaControllerPositionTolerance = 0.5;
    public static final double kDriftCorrectionThetaControllerVelocityTolerance = 0.5;

    public static final PIDConstants kTargetAlignmentThetaControllerPIDConstants = new PIDConstants(0.1, 0, 0.01);
    public static final double kTargetAlignmentThetaControllerPositionTolerance = 0.5;
    public static final double kTargetAlignmentThetaControllerVelocityTolerance = 0.5;

    public static final double kDriveInputLimiter = 0.6;
    public static final double kDriveInputRateLimit = 0.5;

    public static final com.pathplanner.lib.util.PIDConstants kPathFollowerTranslationPIDConstants = new com.pathplanner.lib.util.PIDConstants(0.5, 0, 0);
    public static final com.pathplanner.lib.util.PIDConstants kPathFollowerRotationPIDConstants = new com.pathplanner.lib.util.PIDConstants(0.7, 0, 0);
    public static final PathConstraints kPathFindingConstraints = new PathConstraints(3.0, 3.0, 540.00, 720.00);

    public static final class SwerveModule {
      public static final double kOffsetFrontLeft = -Math.PI / 2;
      public static final double kOffsetFrontRight = 0;
      public static final double kOffsetRearLeft = Math.PI;
      public static final double kOffsetRearRight = Math.PI / 2;

      public static final int kDrivingMotorPinionTeeth = 14;
      public static final double kFreeSpeedRpm = 6238.73054766; //5676
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

  public static final class Feeder {
    public static final int kArmMotorCANId = 15;
    public static final int kRollerMotorCANId = 14;

    public static final int kArmMotorCurrentLimit = 60;
    public static final double kArmMotorMinOutput = -0.5;
    public static final double kArmMotorMaxOutput = 0.5;
    public static final IdleMode kArmMotorIdleMode = IdleMode.kBrake;
    public static final PIDConstants kArmMotorPIDConstants = new PIDConstants(0.05, 0, 0);
    public static final double kArmMotorForwardSoftLimit = 19.0; // TODO: recalibrate
    public static final double kArmMotorReverseSoftLimit = 1.0; // TODO: recalibrate
    public static final double kArmMotorPositionConversionFactor = 1.0 / 3.0;
    public static final double kArmMotorVelocityConversionFactor = kArmMotorPositionConversionFactor / 60.0;
    public static final double kArmMotorSmartMotionMaxVelocity = (33.0 / kArmMotorPositionConversionFactor) * 60;
    public static final double kArmMotorSmartMotionMaxAccel = 100.0 / kArmMotorVelocityConversionFactor;
    
    public static final int kRollerMotorCurrentLimit = 60;
    public static final double kRollerMotorMaxOutput = 0.75;
    public static final IdleMode kRollerMotorIdleMode = IdleMode.kBrake;
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
    public static final double kArmMotorForwardSoftLimit = 15;
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

    // TODO: move speeds and positions for game play to game constants

    public static final RollerSpeeds kWarmupLauncherSpeeds = new RollerSpeeds(0.65, 0.65);
    public static final RollerSpeeds kAmpLauncherSpeeds = new RollerSpeeds(0.35, 0.35);
    
    public static final double kArmPositionIntake = 13.0;
    public static final double kArmPositionAmp = 11.4;
    public static final double kArmPositionSubwoofer = 13.10; // 1.35m
    public static final double kArmPositionShortRange = 9.0; // 1.84m
    public static final double kArmPositionMidRange = 7.0; // 2.78m
    public static final double kArmPositionLongRange = 4.20; // 5.37m

    public static final double[] kDistances = new double[] { 1.35, 1.84, 2.78, 5.37 };
    public static final double[] kPositions = new double[] { 13.10, 9.0, 7.0, 4.20 };
  }

  public static final class Climber {
    public static final int kArmMotorCANId = 16;
    public static final int kRollerMotorCANId = 17;

    public static final int kArmMotorCurrentLimit = 60;
    public static final double kArmMotorMinOutput = -0.8; // TODO: update with testing
    public static final double kArmMotorMaxOutput = 0.8; // TODO: update with testing
    public static final IdleMode kArmMotorIdleMode = IdleMode.kBrake;
    public static final PIDConstants kArmMotorPIDConstants = new PIDConstants(0.0003, 0, 0.00015, 1 / 16.8);
    public static final double kArmMotorForwardSoftLimit = 38.7;
    public static final double kArmMotorReverseSoftLimit = 0.0; 
    public static final double kArmMotorPositionConversionFactor = 1.0 / 9.0; 
    public static final double kArmMotorVelocityConversionFactor = kArmMotorPositionConversionFactor / 60.0;
    public static final double kArmMotorSmartMotionMaxVelocity = (33.0 / kArmMotorPositionConversionFactor) * 60;
    public static final double kArmMotorSmartMotionMaxAccel = 100.0 / kArmMotorVelocityConversionFactor;

    public static final int kRollerMotorCurrentLimit = 60;
    public static final double kRollerMotorMinOutput = -1.0; // TODO: update with testing
    public static final double kRollerMotorMaxOutput = 1.0; // TODO: update with testing
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
            new Translation3d(Units.inchesToMeters(-5), Units.inchesToMeters(-11), Units.inchesToMeters(15.00)),
            new Rotation3d(Units.degreesToRadians(3.2), Units.degreesToRadians(-24.3), Units.degreesToRadians(180))
          )
        ),
        entry(
          "Side",
          new Transform3d(
            new Translation3d(Units.inchesToMeters(-5.5), Units.inchesToMeters(-12), Units.inchesToMeters(23.75)),
            new Rotation3d(0, Units.degreesToRadians(-24.1), Units.degreesToRadians(-87))
          )
        )
      );
      public static final PoseStrategy kPoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
      public static final PoseStrategy kFallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY;
      public static final Matrix<N3, N1> kSingleTagStandardDeviations = VecBuilder.fill(2, 2, 4);
      public static final Matrix<N3, N1> kMultiTagStandardDeviations = VecBuilder.fill(0.5, 0.5, 1);

      public static final double kTargetAlignmentYawCorrection = 3.0;
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

      public static final Pair<Pose2d, Pose2d> kBoundaries = Pair.of(
        new Pose2d(0, 0, null), 
        new Pose2d(kAprilTagFieldLayout.getFieldLength(), kAprilTagFieldLayout.getFieldWidth(), null)
      );

      public static final class Targets {
        public static final double kSpeakerXDelta = 0.3;
        public static final double kSpeakerZDelta = 0.616675;
        public static final Pose3d kAprilTag7 = kAprilTagFieldLayout.getTagPose(7).orElse(new Pose3d());
        public static final Pose3d kBlueSpeaker = new Pose3d(kAprilTag7.getX() + kSpeakerXDelta, kAprilTag7.getY(), kAprilTag7.getZ() + kSpeakerZDelta, kAprilTag7.getRotation());
        public static final Pose3d kAprilTag4 = kAprilTagFieldLayout.getTagPose(4).orElse(new Pose3d());
        public static final Pose3d kRedSpeaker = new Pose3d(kAprilTag4.getX() - kSpeakerXDelta, kAprilTag4.getY(), kAprilTag4.getZ() + kSpeakerZDelta, kAprilTag4.getRotation());

        public static final double kAmpZDelta = -0.4572;
        public static final Pose3d kAprilTag5 = kAprilTagFieldLayout.getTagPose(5).orElse(new Pose3d());
        public static final Pose3d kBlueAmp = new Pose3d(kAprilTag5.getX(), kAprilTag5.getY(), kAprilTag5.getZ() + kAmpZDelta, kAprilTag5.getRotation());
        public static final Pose3d kAprilTag6 = kAprilTagFieldLayout.getTagPose(6).orElse(new Pose3d());
        public static final Pose3d kRedAmp = new Pose3d(kAprilTag6.getX(), kAprilTag6.getY(), kAprilTag6.getZ() + kAmpZDelta, kAprilTag6.getRotation());

        public static final double kSourceZ = 0.93;
      }

      public static final class AutoWaypoints {
        public static final NotePoses kNotePreload1Poses = new NotePoses(new Pose2d(), new Pose2d(1.84, 6.70, Rotation2d.fromDegrees(0)));
        public static final Pose2d kScoreNotePreload2 = new Pose2d(1.84, 5.38, Rotation2d.fromDegrees(0));
        public static final Pose2d kScoreNotePreload3 = new Pose2d(1.84, 4.00, Rotation2d.fromDegrees(0));

        public static final NotePoses kNote1Poses = new NotePoses(
          new Pose2d(2.78, 6.90, Rotation2d.fromDegrees(0)), 
          new Pose2d(2.78, 6.90, Rotation2d.fromDegrees(0)));

        public static final NotePoses kNote2Poses = new NotePoses(
          new Pose2d(2.78, 5.38, Rotation2d.fromDegrees(0)), 
          new Pose2d(2.78, 5.38, Rotation2d.fromDegrees(0)));

        public static final NotePoses kNote3Poses = new NotePoses(
          new Pose2d(2.52, 4.05, Rotation2d.fromDegrees(0)), 
          new Pose2d(2.52, 4.05, Rotation2d.fromDegrees(0)));

        public static final NotePoses kNote4Poses = new NotePoses(
          new Pose2d(7.87, 7.03, Rotation2d.fromDegrees(0)), 
          new Pose2d(5.37, 6.15, Rotation2d.fromDegrees(0)));

        public static final NotePoses kNote5Poses = new NotePoses(
          new Pose2d(7.87, 5.37, Rotation2d.fromDegrees(0)), 
          new Pose2d(5.37, 6.15, Rotation2d.fromDegrees(0)));

        public static final NotePoses kNote6Poses = new NotePoses(
          new Pose2d(7.87, 3.70, Rotation2d.fromDegrees(0)), 
          new Pose2d(5.37, 6.15, Rotation2d.fromDegrees(0)));

        public static final NotePoses kNote7Poses = new NotePoses(
          new Pose2d(7.87, 2.03, Rotation2d.fromDegrees(0)), 
          new Pose2d(5.37, 1.90, Rotation2d.fromDegrees(0)));

        public static final NotePoses kNote8Poses = new NotePoses(
          new Pose2d(7.87, 7.03, Rotation2d.fromDegrees(0)), 
          new Pose2d(5.37, 1.90, Rotation2d.fromDegrees(0)));

        // TODO: validate - same as note 4 scoring position
      }
    }
  }
}
