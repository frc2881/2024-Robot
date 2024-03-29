package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
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
import frc.robot.lib.common.Enums.LauncherAlignmentTarget;
import frc.robot.lib.common.Enums.LightsMode;
import frc.robot.lib.common.Utils;
import frc.robot.lib.controllers.GameController;
import frc.robot.lib.controllers.LightsController;
import frc.robot.lib.sensors.BeamBreakSensor;
import frc.robot.lib.sensors.GyroSensor;
import frc.robot.lib.sensors.ObjectSensor;
import frc.robot.lib.sensors.PoseSensor;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherArmSubsystem;
import frc.robot.subsystems.LauncherRollerSubsystem;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.subsystems.TrapBlowerSubsystem;

public class RobotContainer {
  private final PowerDistribution m_powerDistribution;
  private final GameController m_driverController;
  private final GameController m_operatorController;
  private final GyroSensor m_gyroSensor;
  private final List<PoseSensor> m_poseSensors;
  private final BeamBreakSensor m_launcherBottomBeamBreakSensor;
  private final BeamBreakSensor m_launcherTopBeamBreakSensor;
  private final ObjectSensor m_objectSensor;
  private final DriveSubsystem m_driveSubsystem;
  private final PoseSubsystem m_poseSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final LauncherArmSubsystem m_launcherArmSubsystem;
  private final LauncherRollerSubsystem m_launcherRollerSubsystem;
  private final ClimberSubsystem m_climberSubsystem;
  private final TrapBlowerSubsystem m_trapBlowerSubsystem;
  private final LightsController m_lightsController;
  private final GameCommands m_gameCommands;
  private final AutoCommands m_autoCommands;
  private final SendableChooser<Supplier<Command>> m_autoChooser;

  public RobotContainer() {
    // HARDWARE ========================================
    m_powerDistribution = new PowerDistribution(1, ModuleType.kRev);

    // INPUT CONTROLLERS ========================================
    m_driverController = new GameController(Constants.Controllers.kDriverControllerPort);
    m_operatorController = new GameController(Constants.Controllers.kOperatorControllerPort);

    // OUTPUT CONTROLLERS ========================================
    m_lightsController = new LightsController();

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
    m_launcherBottomBeamBreakSensor = new BeamBreakSensor(
      Constants.Sensors.BeamBreak.LauncherBottom.kSensorName,
      Constants.Sensors.BeamBreak.LauncherBottom.kChannel
    );
    m_launcherTopBeamBreakSensor = new BeamBreakSensor(
      Constants.Sensors.BeamBreak.LauncherTop.kSensorName,
      Constants.Sensors.BeamBreak.LauncherTop.kChannel
    );
    m_objectSensor = new ObjectSensor(
      Constants.Sensors.Object.kCameraName,
      Constants.Sensors.Object.kObjectName
    );

    // SUBSYSTEMS ========================================
    m_climberSubsystem = new ClimberSubsystem();
    m_intakeSubsystem = new IntakeSubsystem();
    m_launcherArmSubsystem = new LauncherArmSubsystem();
    m_launcherRollerSubsystem = new LauncherRollerSubsystem();
    m_driveSubsystem = new DriveSubsystem(m_gyroSensor::getHeading);
    m_trapBlowerSubsystem = new TrapBlowerSubsystem();
    m_poseSubsystem = new PoseSubsystem(m_poseSensors, m_gyroSensor::getRotation2d, m_driveSubsystem::getSwerveModulePositions);
    
    // COMMANDS ========================================
    m_gameCommands = new GameCommands(
      m_launcherBottomBeamBreakSensor, 
      m_launcherTopBeamBreakSensor,  
      m_driveSubsystem, 
      m_poseSubsystem, 
      m_intakeSubsystem, 
      m_launcherArmSubsystem, 
      m_launcherRollerSubsystem, 
      m_climberSubsystem, 
      m_trapBlowerSubsystem,
      m_driverController, 
      m_operatorController
    );

    m_autoCommands = new AutoCommands(m_gameCommands);
    m_autoChooser = new SendableChooser<Supplier<Command>>();

    configureBindings();
    configureTriggers();
    configureAutos();
  }

  private void configureBindings() {
    // DRIVER ========================================
    m_driveSubsystem.setDefaultCommand(m_driveSubsystem.driveWithControllerCommand(m_driverController::getLeftY, m_driverController::getLeftX, m_driverController::getRightX));
    m_driverController.leftTrigger().whileTrue(m_gameCommands.alignLauncherForShuttleCommand());
    m_driverController.rightTrigger().whileTrue(m_gameCommands.runIntakeCommand());
    m_driverController.leftBumper().whileTrue(m_gameCommands.runEjectCommand());
    m_driverController.rightBumper().whileTrue(m_gameCommands.runLauncherForShuttleCommand());
    m_driverController.leftStick().whileTrue(m_driveSubsystem.setLockedCommand());
    m_driverController.rightStick().whileTrue(m_gameCommands.alignRobotToTargetCommand());
    // m_driverController.povLeft().whileTrue(Commands.none());
    // m_driverController.povUp().whileTrue(Commands.none()); 
    // m_driverController.povRight().whileTrue(Commands.none());
    // m_driverController.povDown().whileTrue(Commands.none()); 
    m_driverController.a().whileTrue(m_gameCommands.alignRobotToTargetCommand());
    m_driverController.b().whileTrue(m_gameCommands.runReloadCommand());
    // m_driverController.y().whileTrue(m_trapBlowerSubsystem.runCommand());
    //m_driverController.x().whileTrue(m_gameCommands.runLauncherForShuttleCommand());
    m_driverController.start().onTrue(m_gyroSensor.calibrateCommand());
    m_driverController.back().onTrue(m_gyroSensor.resetCommand());

    // OPERATOR ========================================
    m_launcherArmSubsystem.setDefaultCommand(m_launcherArmSubsystem.alignManualCommand(m_operatorController::getLeftY));
    m_climberSubsystem.setDefaultCommand(m_climberSubsystem.moveArmManualCommand(m_operatorController::getRightY));
    m_operatorController.rightTrigger().whileTrue(m_gameCommands.runLauncherForTargetCommand());
    m_operatorController.rightBumper().whileTrue(m_gameCommands.runLauncherForTrapCommand());
    m_operatorController.leftTrigger().whileTrue(m_gameCommands.alignLauncherToSpeakerCommand(true));
    m_operatorController.leftBumper()
      .whileTrue(m_gameCommands.alignLauncherToTrapOrAmpCommand())
      .onTrue(m_gameCommands.setLauncherTarget(LauncherAlignmentTarget.Amp)
        .onlyIf(() -> m_gameCommands.m_launcherTarget != LauncherAlignmentTarget.Trap))
      .onFalse(m_gameCommands.setLauncherTarget(LauncherAlignmentTarget.Speaker));
    // m_operatorController.leftStick().whileTrue(Commands.none());
    // m_operatorController.rightStick().whileTrue(Commands.none());
    m_operatorController.povLeft().whileTrue(m_gameCommands.alignLauncherToPositionCommand(Constants.Launcher.kArmPositionLongRange, true));
    m_operatorController.povUp().whileTrue(m_gameCommands.alignLauncherToPositionCommand(Constants.Launcher.kArmPositionMidRange, true));  
    m_operatorController.povRight().whileTrue(m_gameCommands.alignLauncherToPositionCommand(Constants.Launcher.kArmPositionShortRange, true));  
    m_operatorController.povDown().whileTrue(m_gameCommands.alignLauncherToPositionCommand(Constants.Launcher.kArmPositionSubwoofer, true));
    //m_operatorController.a().whileTrue(m_gameCommands.alignLauncherToSpeakerCommand(true)); 
    // m_operatorController.y().whileTrue();
    // m_operatorController.b().whileTrue();
    m_operatorController.x().whileTrue(m_trapBlowerSubsystem.runCommand())
      .onTrue(m_gameCommands.setLauncherTarget(LauncherAlignmentTarget.Trap))
      .onFalse(m_gameCommands.setLauncherTarget(LauncherAlignmentTarget.Speaker));
    // m_operatorController.x().whileTrue(m_gameCommands.alignLauncherToTrapCommand(false));
    m_operatorController.back().whileTrue(m_launcherArmSubsystem.resetCommand());
    m_operatorController.start().whileTrue(m_climberSubsystem.resetCommand());
  }

  private void configureTriggers() {
    new Trigger(
      () -> m_launcherBottomBeamBreakSensor.hasTarget())
      .whileTrue(Commands.run(() -> { 
        LightsMode lightsMode = LightsMode.IntakeReady;
        if (m_launcherTopBeamBreakSensor.hasTarget()) {
          lightsMode = LightsMode.IntakeNotReady;
        } else {
          if (m_driveSubsystem.isAlignedToTarget() && m_launcherArmSubsystem.isAlignedToTarget()) {
            lightsMode = LightsMode.LaunchReady;
          }
        }
        m_lightsController.setLightsMode(lightsMode);        
      }).ignoringDisable(true))
      .onFalse(Commands.runOnce(() -> { 
        m_lightsController.setLightsMode(LightsMode.Default); 
      }).ignoringDisable(true)
    );
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

    PathfindingCommand.warmupCommand().schedule();

    m_autoChooser.setDefaultOption("None", Commands::none);

    m_autoChooser.addOption("[ 1 ] 0", m_autoCommands::auto0);
    m_autoChooser.addOption("[ 1 ] 0_1", m_autoCommands::auto10_1);
    m_autoChooser.addOption("[ 1 ] _0_1", m_autoCommands::auto1_0_1);
    m_autoChooser.addOption("[ 1 ] _0_1_41", m_autoCommands::auto1_0_1_41);
    m_autoChooser.addOption("[ 1 ] _0_1_41_51", m_autoCommands::auto1_0_1_41_51);
    m_autoChooser.addOption("[ 1 ] _0_1_51", m_autoCommands::auto1_0_1_51);
    m_autoChooser.addOption("[ 1 ] _0_1_51_41", m_autoCommands::auto1_0_1_51_41);
    m_autoChooser.addOption("[ 1 ] _0_1_51_62", m_autoCommands::auto1_0_1_51_62);

    m_autoChooser.addOption("[ 2 ] 0", m_autoCommands::auto0);
    m_autoChooser.addOption("[ 2 ] 0_2", m_autoCommands::auto20_2);
    m_autoChooser.addOption("[ 2 ] _0_2", m_autoCommands::auto2_0_2);
    m_autoChooser.addOption("[ 2 ] _0_2_62", m_autoCommands::auto2_0_2_62);
    m_autoChooser.addOption("[ 2 ] _0_2_62_51", m_autoCommands::auto2_0_2_62_51);
    m_autoChooser.addOption("[ 2 ] _0_2_62_72", m_autoCommands::auto2_0_2_62_72);
    m_autoChooser.addOption("[ 2 ] _0_2_72", m_autoCommands::auto2_0_2_72);
    m_autoChooser.addOption("[ 2 ] _0_2_72_62", m_autoCommands::auto2_0_2_72_62);

    m_autoChooser.addOption("[ 3 ] 0", m_autoCommands::auto0);
    m_autoChooser.addOption("[ 3 ] 0_3", m_autoCommands::auto30_3);
    m_autoChooser.addOption("[ 3 ] 0_73", m_autoCommands::auto30_73);
    m_autoChooser.addOption("[ 3 ] 0_83", m_autoCommands::auto30_83);
    m_autoChooser.addOption("[ 3 ] 0_73_83", m_autoCommands::auto30_73_83);
    m_autoChooser.addOption("[ 3 ] 0_83_73", m_autoCommands::auto30_83_73);
    m_autoChooser.addOption("[ 3 ] _0_3", m_autoCommands::auto3_0_3);
    // TOOD: create auto3_0_3_72
    // TODO: create auto3_0_3_73
    // TODO: create auto3_0_3_72_62
    // TODO: create auto3_0_3_73_83
    m_autoChooser.addOption("[ 3 ] _0_3_82", m_autoCommands::auto3_0_3_82);
    m_autoChooser.addOption("[ 3 ] _0_3_83", m_autoCommands::auto3_0_3_83);
    m_autoChooser.addOption("[ 3 ] _0_3_82_62", m_autoCommands::auto3_0_3_82_62);
    m_autoChooser.addOption("[ 3 ] _0_3_82_72", m_autoCommands::auto3_0_3_82_72);
    m_autoChooser.addOption("[ 3 ] _0_3_83_62", m_autoCommands::auto3_0_3_83_62);
    m_autoChooser.addOption("[ 3 ] _0_3_83_72", m_autoCommands::auto3_0_3_83_72);
    m_autoChooser.addOption("[ 3 ] _0_3_83_73", m_autoCommands::auto3_0_3_83_73);

    SmartDashboard.putData("Robot/Auto/Command", m_autoChooser);
  }

  public Command getSelectedAutoCommand() {
    return m_autoChooser.getSelected().get();
  }

  public void resetRobot() {
    m_driveSubsystem.reset();
    m_intakeSubsystem.reset();
    m_launcherRollerSubsystem.reset();
    m_launcherArmSubsystem.reset();
    m_climberSubsystem.reset();
  }

  public void autonomousInit() {
    resetRobot();
    m_gyroSensor.calibrate();
  }

  public void teleopInit() {
    resetRobot();
    m_gyroSensor.reset(Utils.wrapAngle(m_poseSubsystem.getPose().getRotation().getDegrees()));
  }

  public void testInit() {
    resetRobot();
  }

  public void simulationInit() {
    resetRobot();
  }

  public void updateTelemetry() {
    m_gyroSensor.updateTelemetry();
    m_poseSensors.forEach(poseSensor -> poseSensor.updateTelemetry());
    m_launcherBottomBeamBreakSensor.updateTelemetry();
    m_launcherTopBeamBreakSensor.updateTelemetry();
    m_objectSensor.updateTelemetry();

    SmartDashboard.putNumber("Robot/Power/TotalCurrent", m_powerDistribution.getTotalCurrent());

    SmartDashboard.putBoolean(
      "Robot/HasInitialReset", 
      m_launcherArmSubsystem.hasInitialReset() &&
      m_climberSubsystem.hasInitialReset()
    );
  }
}
