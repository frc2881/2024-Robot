package frc.robot;

import static java.util.Map.entry;

import java.util.Map;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.lib.common.PIDConstants;

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

    public static final PIDConstants kTargetAlignmentThetaControllerPIDConstants = new PIDConstants(0.1, 0, 0.01); // TODO: calibrate with testing (faster?)
    public static final double kTargetAlignmentThetaControllerPositionTolerance = 0.5;
    public static final double kTargetAlignmentThetaControllerVelocityTolerance = 0.5;

    public static final double kDriveInputLimiter = 0.6;
    public static final double kDriveInputRateLimit = 0.5;

    public static final com.pathplanner.lib.util.PIDConstants kPathFollowerTranslationPIDConstants = new com.pathplanner.lib.util.PIDConstants(5, 0, 0); // TODO: update after testing
    public static final com.pathplanner.lib.util.PIDConstants kPathFollowerRotationPIDConstants = new com.pathplanner.lib.util.PIDConstants(5, 0, 0); // TODO: update after testing

    public static final class SwerveModule {
      public static final double kOffsetFrontLeft = -Math.PI / 2;
      public static final double kOffsetFrontRight = 0;
      public static final double kOffsetRearLeft = Math.PI;
      public static final double kOffsetRearRight = Math.PI / 2;

      public static final int kDrivingMotorPinionTeeth = 14;
      public static final double kFreeSpeedRpm = 5676;
      public static final double kWheelDiameterMeters = Units.inchesToMeters(3.0);
      public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
      public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
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
    public static final PIDConstants kArmMotorPIDConstants = new PIDConstants(0.1, 0, 0); // TODO: calibrate to be more gentle with arm (convert to smart motion in subsytem?)
    public static final double kArmMotorForwardSoftLimit = 19.0; // TODO: recalibrate soft limits
    public static final double kArmMotorReverseSoftLimit = 0.0; // TODO: recalibrate soft limits

    public static final int kRollerMotorCurrentLimit = 60;
    public static final double kRollerMotorMaxOutput = 0.75;
    public static final IdleMode kRollerMotorIdleMode = IdleMode.kBrake;
  }

  public static final class Intake {
    public static final int kTopBeltMotorCANId = 18;
    public static final int kBottomBeltMotorCANId = 19;
    public static final int kRollerMotorCANId = 20;

    public static final int kTopBeltMotorCurrentLimit = 60;
    public static final double kTopBeltMotorMinOutput = -0.5; // TODO: update after testing
    public static final double kTopBeltMotorMaxOutput = 0.5; // TODO: update after testing
    public static final IdleMode kTopBeltMotorIdleMode = IdleMode.kBrake;

    public static final int kBottomBeltMotorCurrentLimit = 60;
    public static final double kBottomBeltMotorMinOutput = -0.5; // TODO: update after testing
    public static final double kBottomBeltMotorMaxOutput = 0.5; // TODO: update after testing
    public static final IdleMode kBottomBeltMotorIdleMode = IdleMode.kCoast;

    public static final int kRollerMotorCurrentLimit = 60;
    public static final double kRollerMotorMinOutput = -0.5; // TODO: update after testing
    public static final double kRollerMotorMaxOutput = 0.5; // TODO: update after testing
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

    // TODO: calculate and set as constant the launcher position to angle conversion factor to use for dynamic adjustment based on distance from target
    // TODO: recalibrate position measurements/angles while maintaining/holding position (keep holding the controller button for the set position)
    // TODO: proprtion calculation should be rate of angle change per unit of position change based on physical measurements
    public static final double kArmPositionFromTargetPitchConversionFactor = (12.0 - 11.25) / (55.3 - 52.2);

    public static final double kArmPositionIntake = 14.0; // TODO: get degrees 
    public static final double kArmPositionSubwoofer = 13.10; // TODO: configure on field (12.0 measured at 55.3 degrees, 13.10 measured at 58.0 degrees)
    public static final double kArmPositionMidRange = 7.5; // TODO: configure on field (11.25 measured at 52.2 degrees)
    public static final double kArmPositionLongRange = 4.20; // TODO: configure on field (4.20 measure at 23.8 degrees)
    public static final double kArmPositionAmp = 11.4; // TODO: configure on field
  }

  public static final class Climber {
    public static final int kArmMotorCANId = 16;
    public static final int kRollerMotorCANId = 17;

    public static final int kArmMotorCurrentLimit = 60;
    public static final double kArmMotorMinOutput = -0.2; // TODO: update after testing
    public static final double kArmMotorMaxOutput = 0.2; // TODO: update after testing
    public static final IdleMode kArmMotorIdleMode = IdleMode.kBrake;
    public static final PIDConstants kArmMotorPIDConstants = new PIDConstants(0.0003, 0, 0.00015, 1 / 16.8);
    public static final double kArmMotorForwardSoftLimit = 100; // TODO: update
    public static final double kArmMotorReverseSoftLimit = 0.5; // TODO: update
    public static final double kArmMotorPositionConversionFactor = 1.0 / 3.0; // TODO: update (gear ratio for the leadscrew that converts rotations to inches of extension)
    public static final double kArmMotorVelocityConversionFactor = kArmMotorPositionConversionFactor / 60.0;
    public static final double kArmMotorSmartMotionMaxVelocity = (33.0 / kArmMotorPositionConversionFactor) * 60;
    public static final double kArmMotorSmartMotionMaxAccel = 100.0 / kArmMotorVelocityConversionFactor;

    public static final int kRollerMotorCurrentLimit = 60;
    public static final double kRollerMotorMinOutput = -1.0; // TODO: update after testing
    public static final double kRollerMotorMaxOutput = 1.0; // TODO: update after testing
    public static final IdleMode kRollerMotorIdleMode = IdleMode.kBrake;

    //37.7 is position for arm to lock
  }

  public static final class Sensors {
    public static final class Gyro {
      public static final IMUAxis kIMUAxisYaw = IMUAxis.kZ;
      public static final IMUAxis kIMUAxisRoll = IMUAxis.kY;
      public static final IMUAxis kIMUAxisPitch = IMUAxis.kX;
      public static final SPI.Port kSPIPort = SPI.Port.kOnboardCS0;
      public static final CalibrationTime kCalibrationTime = CalibrationTime._2s;
    }

    public static final class Pose {
      // TODO: enable and configure cameras and transforms after mounting to robot
      public static final Map<String, Transform3d> kPoseSensors = Map.ofEntries(
        entry(
          "Rear",
          new Transform3d(
            new Translation3d(Units.inchesToMeters(-5), Units.inchesToMeters(-11), Units.inchesToMeters(15)), //  23.0 24.2
            new Rotation3d(Units.degreesToRadians(3.73), Units.degreesToRadians(-90 + 24.2), Units.degreesToRadians(180))
          )
        ),
        entry(
          "Side",
          new Transform3d(
            new Translation3d(Units.inchesToMeters(-5.5), Units.inchesToMeters(-11.5), Units.inchesToMeters(24)),
            new Rotation3d(0, Units.degreesToRadians(-90 + 23.0), Units.degreesToRadians(-90))
          )
        )
      );
      public static final PoseStrategy kPoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
      public static final PoseStrategy kFallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY;
      public static final Matrix<N3, N1> kSingleTagStandardDeviations = VecBuilder.fill(2, 2, 4);
      public static final Matrix<N3, N1> kMultiTagStandardDeviations = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static final class Object {
      public static final String kCameraName = "Front";
      public static final double kObjectRangeYaw = 10.0; // TODO: update after testing
    }

    public static final class Distance {
      public static final class Intake {
        public static final String kSensorName = "Intake";
        public static final double kMinTargetDistance = 0;
        public static final double kMaxTargetDistance = 25; // TODO: update after testing note detection within intake
      }
      public static final class Launcher {
        public static final String kSensorName = "Launcher";
        public static final double kMinTargetDistance = 0;
        public static final double kMaxTargetDistance = 25; // TODO: update after testing note detection within launcher
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

      // TODO: define proper launch zones based on testing - may only need a minimum distance and rotation angle defined for speaker and amp targets instead of bounding boxes
      public static final class LaunchZones {
        public static final Pair<Pose2d, Pose2d> kSpeaker = Pair.of(
          new Pose2d(0, 0, null), 
          new Pose2d(0, 0, null)
        );
        public static final Pair<Pose2d, Pose2d> kAmp = Pair.of(
          new Pose2d(0, 0, null), 
          new Pose2d(0, 0, null)
        );
      }

      public static final class Targets {
        public static final double kSpeakerXDelta = 0.3; // TODO: validate this through testing
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
    }
  }
}
