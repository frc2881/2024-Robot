package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.ejml.dense.row.decompose.TriangularSolver_CDRM;

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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.GameCommands;
import frc.robot.lib.common.Enums.DriveDriftCorrection;
import frc.robot.lib.common.Enums.DriveOrientation;
import frc.robot.lib.common.Enums.DriveSpeedMode;
import frc.robot.lib.controllers.GameController;
import frc.robot.lib.controllers.LightsController;
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
    m_driverController.x().whileTrue(m_driveSubsystem.setLockedCommand());
    m_driverController.rightTrigger().whileTrue(m_gameCommands.runFrontIntakeCommand());
    m_driverController.leftTrigger().whileTrue(m_gameCommands.runRearIntakeCommand());
    m_driverController.rightTrigger().and(m_driverController.leftTrigger()).whileTrue(m_gameCommands.runEjectIntakeCommand());
    m_driverController.start().onTrue(m_gyroSensor.resetCommand());

    // OPERATOR ========================================
    m_operatorController.a().whileTrue(m_gameCommands.alignLauncherToSpeakerCommand());
    m_operatorController.x().onTrue(m_feederSubsystem.runFeederCommand()).onFalse(m_feederSubsystem.stopFeederCommand());
    m_operatorController.rightTrigger().whileTrue(m_gameCommands.runLauncherCommand()); // TODO: run launcher game command to score note (once launcher is aligned to target by both driver and operator)
    m_operatorController.start().whileTrue(m_feederSubsystem.resetCommand()); // TODO: create parallel game command for resetting feeder, launcher, and arm mechanisms in parallel with one button
    m_operatorController.back().whileTrue(m_launcherSubsystem.resetCommand());
    m_operatorController.b().whileTrue(m_gameCommands.alignLauncherCommand());
    
    new Trigger(() -> Math.abs(m_operatorController.getLeftY()) > 0.1)
      .whileTrue(m_gameCommands.tiltLauncherCommand(m_operatorController::getLeftY));

    new Trigger(() -> Math.abs(m_operatorController.getRightY()) > 0.1)
      .whileTrue(m_gameCommands.moveArmCommand(m_operatorController::getRightY));

    // DASHBOARD ========================================
    SendableChooser<DriveSpeedMode> driveSpeedModeChooser = new SendableChooser<DriveSpeedMode>();
    driveSpeedModeChooser.setDefaultOption(DriveSpeedMode.Competition.toString(), DriveSpeedMode.Competition);
    driveSpeedModeChooser.addOption(DriveSpeedMode.Training.toString(), DriveSpeedMode.Training);
    driveSpeedModeChooser.onChange(speedMode -> m_driveSubsystem.setSpeedMode(speedMode));
    SmartDashboard.putData("Robot/Drive/SpeedMode", driveSpeedModeChooser);

    SendableChooser<DriveOrientation> driveOrientationChooser = new SendableChooser<DriveOrientation>();
    driveOrientationChooser.setDefaultOption(DriveOrientation.Field.toString(), DriveOrientation.Field);
    driveOrientationChooser.addOption(DriveOrientation.Robot.toString(), DriveOrientation.Robot);
    driveOrientationChooser.onChange(orientation -> m_driveSubsystem.setOrientation(orientation));
    SmartDashboard.putData("Robot/Drive/Orientation", driveOrientationChooser);

    SendableChooser<DriveDriftCorrection> driveDriftCorrectionChooser = new SendableChooser<DriveDriftCorrection>();
    driveDriftCorrectionChooser.setDefaultOption(DriveDriftCorrection.Enabled.toString(), DriveDriftCorrection.Enabled);
    driveDriftCorrectionChooser.addOption(DriveDriftCorrection.Disabled.toString(), DriveDriftCorrection.Disabled);
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

    //SmartDashboard.putNumber("Robot/Power/TotalCurrent", m_powerDistribution.getTotalCurrent()); // TODO: debug PDH data message call errors
  }
}
