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

public final class Constants {

  public static final class Controllers {
    public static final int kDriverControllerPort = 0; 
    public static final int kOperatorControllerPort = 1; 
    public static final double kDriveInputDeadband = 0.1; 
    public static final double kDriveInputLimiter = 0.6;
    public static final double kDriveInputRateLimit = 0.5;
  }

  public static final class Drive {
    public static final int kFrontLeftDrivingCanId = 3;
    public static final int kFrontLeftTurningCanId = 4;
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightDrivingCanId = 7;
    public static final int kFrontRightTurningCanId = 8;
    public static final int kRearRightDrivingCanId = 9;
    public static final int kRearRightTurningCanId = 10;

    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    public static final double kWheelBase = Units.inchesToMeters(21.5);
    public static final double kDriveBaseRadius = new Translation2d(kWheelBase / 2, kTrackWidth / 2).getNorm();

    public static final double kMaxSpeedMetersPerSecond = 5.7424;
    public static final double kMaxAngularSpeed = 2 * Math.PI;

    public static final double kFrontLeftTurningOffset = -Math.PI / 2;
    public static final double kFrontRightTurningOffset = 0;
    public static final double kRearLeftTurningOffset = Math.PI;
    public static final double kRearRightTurningOffset = Math.PI / 2;

    public static final SwerveDriveKinematics kSwerveDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.Drive.kWheelBase / 2, Constants.Drive.kTrackWidth / 2),
      new Translation2d(Constants.Drive.kWheelBase / 2, -Constants.Drive.kTrackWidth / 2),
      new Translation2d(-Constants.Drive.kWheelBase / 2, Constants.Drive.kTrackWidth / 2),
      new Translation2d(-Constants.Drive.kWheelBase / 2, -Constants.Drive.kTrackWidth / 2)
    );

    public static final double kThetaControllerP = 0.01;
    public static final double kThetaControllerI = 0;
    public static final double kThetaControllerD = 0;
    public static final double kThetaControllerPositionTolerance = 0.5;
    public static final double kThetaControllerVelocityTolerance = 0.5;

    public static final double kPathFollowerTranslationP = 2.5;
    public static final double kPathFollowerTranslationI = 0;
    public static final double kPathFollowerTranslationD = 0;
    public static final double kPathFollowerRotationP = 5;
    public static final double kPathFollowerRotationI = 0;
    public static final double kPathFollowerRotationD = 0;

    public static final class SwerveModule {
      public static final int kDrivingMotorPinionTeeth = 14;
      public static final boolean kTurningEncoderInverted = true;
      public static final double kFreeSpeedRpm = 5676;
      public static final double kDrivingMotorFreeSpeedRps = kFreeSpeedRpm / 60;
      public static final double kWheelDiameterMeters = Units.inchesToMeters(3.0);
      public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
      public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
      public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
      public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction;
      public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0;
      public static final double kTurningEncoderPositionFactor = (2 * Math.PI);
      public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0;
      public static final double kTurningEncoderPositionPIDMinInput = 0;
      public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor;
      public static final double kDrivingP = 0.04;
      public static final double kDrivingI = 0;
      public static final double kDrivingD = 0;
      public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
      public static final double kDrivingMinOutput = -1;
      public static final double kDrivingMaxOutput = 1;
      public static final double kTurningP = 1;
      public static final double kTurningI = 0;
      public static final double kTurningD = 0;
      public static final double kTurningFF = 0;
      public static final double kTurningMinOutput = -1;
      public static final double kTurningMaxOutput = 1;
      public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
      public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;
      public static final int kDrivingMotorCurrentLimit = 50;
      public static final int kTurningMotorCurrentLimit = 20;
    }
  }

  public static final class Feeder {
    public static int kRollerCanId = 14;
    public static int kArmCanId = 15;
    public static double kForwardLimit = 10; // TODO: update
    public static double kReverseLimit = 0.5; // TODO: update
  }

  public static final class Intake {
    public static final int kBeltCanId = 18;
    public static final int kRollerCanId = 19;
  }

  public static final class Launcher {
    public static final int kLeadScrewCanId = 11;
    public static final int kTopRollerCanId = 12;
    public static final int kBottomRollerCanId = 13;

    // Max distance that the launcher can tilt
    public static final double kLeadScrewForwardLimit = 100; // TODO: update
    // Least distance that the launcher can tilt
    public static final double kLeadScrewReverseLimit = 0.5; // TODO: update

    public static final double kRotationsToInches = 1.0 / 3.0; // TODO: update (gear ratio for the leadscrew that converts rotations to inches of extension)
    public static final double kVelocityConversion = kRotationsToInches / 60.0;

    public static final double kLeadScrewP = 0.0003;
    public static final double kLeadScrewD = 0.00015;
    public static final double kLeadScrewFF = 1 / (16.8);

    public static final double kLeadScrewMinOutput = -1.0;
    public static final double kLeadScrewMaxOutput = 1.0;

    public static final double kLaunchUpdateSpeed = 0.5;
  }

  public static final class Arm {
    public static final int kLeadScrewCanId = 16;
    public static final int kRollerCanId = 17;

    // Max distance that the arm can lift
    public static final double kLeadScrewForwardLimit = 100; // TODO: update
    // Least distance that the arm can lower
    public static final double kLeadScrewReverseLimit = 0.5; // TODO: update

    public static final double kRotationsToInches = 1.0 / 3.0; // TODO: update (gear ratio for the arm that converts rotations to inches of extension)
    public static final double kVelocityConversion = kRotationsToInches / 60.0;

    public static final double kLeadScrewP = 0.0003;
    public static final double kLeadScrewD = 0.00015;
    public static final double kLeadScrewFF = 1 / (16.8);

    public static final double kLeadScrewMinOutput = -1.0;
    public static final double kLeadScrewMaxOutput = 1.0;
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
      public static final Map<String, Transform3d> kPoseSensors = Map.ofEntries(
        entry(
          "Rear",
          new Transform3d(
          new Translation3d(Units.inchesToMeters(9.75), 0.00000, Units.inchesToMeters(18.5)),
          new Rotation3d(0, Units.degreesToRadians(35), Units.degreesToRadians(180)))
        )
        // entry(
        //   "Side",
        //   new Transform3d(
        //     new Translation3d(-0.18290, 0.18298, 1.19055),
        //     new Rotation3d(0, Units.degreesToRadians(35), Units.degreesToRadians(180)))
        // )
      );
      public static final PoseStrategy kPoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
      public static final PoseStrategy kFallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY;
      public static final Matrix<N3, N1> kSingleTagStandardDeviations = VecBuilder.fill(2, 2, 4);
      public static final Matrix<N3, N1> kMultiTagStandardDeviations = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static final class Object {
      public static final String kCameraName = "Front";
      public static final Double kObjectRangeYaw = 10.0;
    }

    public static final class Distance {
      public static final String kIntakeSensorName = "Intake";
      public static final String kLauncherSensorName = "Launcher";
    }
  }

  public static final class Game {
    public static final class Field {
      public static final AprilTagFieldLayout kAprilTagFieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

      public static final Pair<Pose2d, Pose2d> kBoundaries = Pair.of(
        new Pose2d(0, 0, null), 
        new Pose2d(kAprilTagFieldLayout.getFieldLength(), kAprilTagFieldLayout.getFieldWidth(), null)
      );

      // TODO: define proper launch zones based on testing
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
        public static final double kSpeakerHeightDelta = 0.616675;
        public static final double kAmpHeightDelta = -0.4572;
        public static final double kSourceHeight = 0.93;

        public static final Pose3d kAprilTag7 = kAprilTagFieldLayout.getTagPose(7).orElse(new Pose3d());
        public static final Pose3d kBlueSpeaker = new Pose3d(kAprilTag7.getX(), kAprilTag7.getY(), kAprilTag7.getZ() + kSpeakerHeightDelta, new Rotation3d());

        public static final Pose3d kAprilTag4 = kAprilTagFieldLayout.getTagPose(4).orElse(new Pose3d());
        public static final Pose3d kRedSpeaker = new Pose3d(kAprilTag4.getX(), kAprilTag4.getY(), kAprilTag4.getZ() + kSpeakerHeightDelta, new Rotation3d());

        public static final Pose3d kAprilTag5 = kAprilTagFieldLayout.getTagPose(5).orElse(new Pose3d());
        public static final Pose3d kBlueAmp = new Pose3d(kAprilTag5.getX(), kAprilTag5.getY(), kAprilTag5.getZ() + kAmpHeightDelta, new Rotation3d());

        public static final Pose3d kAprilTag6 = kAprilTagFieldLayout.getTagPose(6).orElse(new Pose3d());
        public static final Pose3d kRedAmp = new Pose3d(kAprilTag6.getX(), kAprilTag6.getY(), kAprilTag6.getZ() + kAmpHeightDelta, new Rotation3d());
      }
    }
  }
}
