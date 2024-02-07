package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.GameCommands;
import frc.robot.lib.controllers.GameController;
import frc.robot.lib.controllers.LightsController;
import frc.robot.lib.logging.Logger;
import frc.robot.lib.sensors.GyroSensor;
import frc.robot.lib.sensors.ObjectSensor;
import frc.robot.lib.sensors.PoseSensor;
import frc.robot.lib.sensors.DistanceSensor;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseSubsystem;

public class RobotContainer {
  private final PowerDistribution m_powerDistribution;
  private final GameController m_driverController;
  private final GameController m_operatorController;
  private final GyroSensor m_gyroSensor;
  private final List<PoseSensor> m_poseSensors;
  private final DistanceSensor m_intakeDistanceSensor;
  private final DistanceSensor m_launcherDistanceSensor;
  private final ObjectSensor m_objectSensor;
  private final DriveSubsystem m_driveSubsystem;
  private final PoseSubsystem m_poseSubsystem;
  private final FeederSubsystem m_feederSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final LauncherSubsystem m_launcherSubsystem;
  private final ArmSubsystem m_armSubsystem;
  private final LightsController m_lightsController;
  private final GameCommands m_gameCommands;
  private final AutoCommands m_autoCommands;
  private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
    // HARDWARE ========================================
    m_powerDistribution = new PowerDistribution(1, ModuleType.kRev);

    // INPUT CONTROLLERS ========================================
    m_driverController = new GameController(Constants.Controllers.kDriverControllerPort);
    m_operatorController = new GameController(Constants.Controllers.kOperatorControllerPort);

    // SENSORS ========================================
    m_gyroSensor = new GyroSensor(
      Constants.Sensors.Gyro.kIMUAxisYaw, 
      Constants.Sensors.Gyro.kIMUAxisPitch, 
      Constants.Sensors.Gyro.kIMUAxisRoll, 
      Constants.Sensors.Gyro.kSPIPort, 
      Constants.Sensors.Gyro.kCalibrationTime
    );
    m_poseSensors = new ArrayList<PoseSensor>();
    Constants.Sensors.Pose.kPoseSensors.forEach((cameraName, cameraTransform) -> {
      m_poseSensors.add(new PoseSensor(
        cameraName, 
        cameraTransform, 
        Constants.Sensors.Pose.kPoseStrategy, 
        Constants.Sensors.Pose.kFallbackPoseStrategy, 
        Constants.Sensors.Pose.kSingleTagStandardDeviations, 
        Constants.Sensors.Pose.kMultiTagStandardDeviations, 
        Constants.Game.Field.kAprilTagFieldLayout
      ));
    });
    m_intakeDistanceSensor = new DistanceSensor(
      Constants.Sensors.Distance.Intake.kSensorName,
      Constants.Sensors.Distance.Intake.kMinTargetDistance,
      Constants.Sensors.Distance.Intake.kMaxTargetDistance
    );
    m_launcherDistanceSensor = new DistanceSensor(
      Constants.Sensors.Distance.Launcher.kSensorName,
      Constants.Sensors.Distance.Launcher.kMinTargetDistance,
      Constants.Sensors.Distance.Launcher.kMaxTargetDistance
    );
    m_objectSensor = new ObjectSensor(Constants.Sensors.Object.kCameraName);
    
    // SUBSYSTEMS ========================================
    m_driveSubsystem = new DriveSubsystem(m_gyroSensor::getHeading);
    m_poseSubsystem = new PoseSubsystem(m_poseSensors, m_gyroSensor::getRotation2d, m_driveSubsystem::getSwerveModulePositions);
    m_feederSubsystem = new FeederSubsystem();
    m_intakeSubsystem = new IntakeSubsystem();
    m_launcherSubsystem = new LauncherSubsystem();
    m_armSubsystem = new ArmSubsystem();

    // OUTPUT CONTROLLERS ========================================
    m_lightsController = new LightsController();

    // COMMANDS ========================================
    m_gameCommands = new GameCommands(m_gyroSensor, m_intakeDistanceSensor, m_launcherDistanceSensor, m_driveSubsystem, m_poseSubsystem, m_feederSubsystem, m_intakeSubsystem, m_launcherSubsystem, m_armSubsystem, m_lightsController);
    m_autoCommands = new AutoCommands(m_gameCommands, m_gyroSensor, m_objectSensor, m_driveSubsystem, m_poseSubsystem, m_lightsController);
    m_autoChooser = new SendableChooser<Command>();

    configureBindings();
    configureAutos();
  }

  private void configureBindings() {
    // SUBSYSTEMS ========================================
    m_driveSubsystem.setDefaultCommand(m_driveSubsystem.driveWithControllerCommand(m_driverController::getLeftY, m_driverController::getLeftX, m_driverController::getRightX));

    // DRIVER ========================================
    m_driverController.a().whileTrue(m_gameCommands.alignRobotToSpeakerCommand());
    m_driverController.x().onTrue(m_driveSubsystem.toggleLockStateCommand()); // TODO: refactor to while held only vs. toggle for X lock state
    m_driverController.rightTrigger().whileTrue(Commands.none()); // TODO: run intake game command in forward direction (front of robot intake direction for note)
    m_driverController.leftTrigger().whileTrue(Commands.none()); // TODO: run intake game command in backward direction (rear of robot intake direction for note)
    m_driverController.start().onTrue(m_gyroSensor.resetCommand());

    // OPERATOR ========================================
    m_operatorController.a().whileTrue(m_gameCommands.alignLauncherToSpeakerCommand());
    m_operatorController.x().onTrue(m_feederSubsystem.startFeederCommand()).onFalse(m_feederSubsystem.stopFeederCommand());
    m_operatorController.rightTrigger().whileTrue(Commands.none()); // TODO: run launcher game command to score note (once launcher is aligned to target by driver and operator)
    m_operatorController.start().whileTrue(m_feederSubsystem.resetCommand()); // TODO: create parallel game command for resetting feeder, launcher, and arm mechanisms in parallel with one button

    // DASHBOARD ========================================
    SendableChooser<DriveSubsystem.SpeedMode> driveSpeedModeChooser = new SendableChooser<DriveSubsystem.SpeedMode>();
    driveSpeedModeChooser.setDefaultOption(DriveSubsystem.SpeedMode.Competition.toString(), DriveSubsystem.SpeedMode.Competition);
    driveSpeedModeChooser.addOption(DriveSubsystem.SpeedMode.Training.toString(), DriveSubsystem.SpeedMode.Training);
    driveSpeedModeChooser.onChange(speedMode -> m_driveSubsystem.setSpeedMode(speedMode));
    SmartDashboard.putData("Robot/Drive/SpeedMode", driveSpeedModeChooser);

    SendableChooser<DriveSubsystem.Orientation> driveOrientationChooser = new SendableChooser<DriveSubsystem.Orientation>();
    driveOrientationChooser.setDefaultOption(DriveSubsystem.Orientation.Field.toString(), DriveSubsystem.Orientation.Field);
    driveOrientationChooser.addOption(DriveSubsystem.Orientation.Robot.toString(), DriveSubsystem.Orientation.Robot);
    driveOrientationChooser.onChange(orientation -> m_driveSubsystem.setOrientation(orientation));
    SmartDashboard.putData("Robot/Drive/Orientation", driveOrientationChooser);

    SendableChooser<DriveSubsystem.DriftCorrection> driveDriftCorrectionChooser = new SendableChooser<DriveSubsystem.DriftCorrection>();
    driveDriftCorrectionChooser.setDefaultOption(DriveSubsystem.DriftCorrection.Enabled.toString(), DriveSubsystem.DriftCorrection.Enabled);
    driveDriftCorrectionChooser.addOption(DriveSubsystem.DriftCorrection.Disabled.toString(), DriveSubsystem.DriftCorrection.Disabled);
    driveDriftCorrectionChooser.onChange(driftCorrection -> m_driveSubsystem.setDriftCorrection(driftCorrection));
    SmartDashboard.putData("Robot/Drive/DriftCorrection", driveDriftCorrectionChooser);
  }

  private void configureAutos() {
    AutoBuilder.configureHolonomic(
      m_poseSubsystem::getPose, 
      m_poseSubsystem::resetPose, 
      m_driveSubsystem::getSpeeds, 
      m_driveSubsystem::drive, 
      new HolonomicPathFollowerConfig(
        Constants.Drive.kPathFollowerTranslationPIDConstants,
        Constants.Drive.kPathFollowerRotationPIDConstants,
        Constants.Drive.kMaxSpeedMetersPerSecond, 
        Constants.Drive.kDriveBaseRadius, 
        new ReplanningConfig()
      ),
      () -> Robot.getAlliance() == Alliance.Red,
      m_driveSubsystem
    );

    m_autoChooser.setDefaultOption("None", Commands.none());
    m_autoChooser.addOption("Test- 3 Note Auto", m_autoCommands.test3NoteAuto());
    m_autoChooser.addOption("Test- Path", m_autoCommands.testPath2());
    
    SmartDashboard.putData("Robot/Auto/Command", m_autoChooser);
  }

  public Command getSelectedAutoCommand() {
    return m_autoChooser.getSelected();
  }

  public void updateTelemetry() {
    m_gyroSensor.updateTelemetry();
    m_poseSensors.forEach(poseSensor -> poseSensor.updateTelemetry());
    m_intakeDistanceSensor.updateTelemetry();
    m_launcherDistanceSensor.updateTelemetry();
    m_objectSensor.updateTelemetry();

    SmartDashboard.putNumber("Robot/Power/TotalCurrent", m_powerDistribution.getTotalCurrent());
  }
}
