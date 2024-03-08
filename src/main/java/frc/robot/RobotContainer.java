package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.GameCommands;
import frc.robot.lib.common.Enums.DriveDriftCorrection;
import frc.robot.lib.common.Enums.DriveOrientation;
import frc.robot.lib.common.Enums.DriveSpeedMode;
import frc.robot.lib.common.Enums.IntakeLocation;
import frc.robot.lib.common.Enums.MotorDirection;
import frc.robot.lib.controllers.GameController;
import frc.robot.lib.controllers.LightsController;
import frc.robot.lib.sensors.BeamBreakSensor;
import frc.robot.lib.sensors.DistanceSensor;
import frc.robot.lib.sensors.GyroSensor;
import frc.robot.lib.sensors.ObjectSensor;
import frc.robot.lib.sensors.PoseSensor;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherArmSubsystem;
import frc.robot.subsystems.LauncherRollerSubsystem;
import frc.robot.subsystems.PoseSubsystem;

public class RobotContainer {
  private final PowerDistribution m_powerDistribution;
  private final GameController m_driverController;
  private final GameController m_operatorController;
  private final GyroSensor m_gyroSensor;
  private final List<PoseSensor> m_poseSensors;
  private final BeamBreakSensor m_intakeBeamBreakSensor;
  private final BeamBreakSensor m_launcherBottomBeamBreakSensor;
  private final BeamBreakSensor m_launcherTopBeamBreakSensor;
  private final DistanceSensor m_intakeDistanceSensor;
  private final DistanceSensor m_launcherDistanceSensor;
  private final ObjectSensor m_objectSensor;
  private final DriveSubsystem m_driveSubsystem;
  private final PoseSubsystem m_poseSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final LauncherArmSubsystem m_launcherArmSubsystem;
  private final LauncherRollerSubsystem m_launcherRollerSubsystem;
  private final ClimberSubsystem m_climberSubsystem;
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
    m_intakeBeamBreakSensor = new BeamBreakSensor(
      Constants.Sensors.BeamBreak.Intake.kSensorName,
      Constants.Sensors.BeamBreak.Intake.kChannel
    );
    m_launcherBottomBeamBreakSensor = new BeamBreakSensor(
      Constants.Sensors.BeamBreak.LauncherBottom.kSensorName,
      Constants.Sensors.BeamBreak.LauncherBottom.kChannel
    );
    m_launcherTopBeamBreakSensor = new BeamBreakSensor(
      Constants.Sensors.BeamBreak.LauncherTop.kSensorName,
      Constants.Sensors.BeamBreak.LauncherTop.kChannel
    );
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
    m_poseSubsystem = new PoseSubsystem(m_poseSensors, m_gyroSensor::getRotation2d, m_driveSubsystem::getSwerveModulePositions);
    
    // COMMANDS ========================================
    m_gameCommands = new GameCommands(
      m_gyroSensor, 
      m_intakeBeamBreakSensor, 
      m_launcherBottomBeamBreakSensor, 
      m_launcherTopBeamBreakSensor, 
      m_intakeDistanceSensor, 
      m_launcherDistanceSensor, 
      m_driveSubsystem, 
      m_poseSubsystem, 
      m_intakeSubsystem, 
      m_launcherArmSubsystem, 
      m_launcherRollerSubsystem, 
      m_climberSubsystem, 
      m_driverController, 
      m_operatorController, 
      m_lightsController
    );

    m_autoCommands = new AutoCommands(
      m_gameCommands, 
      m_gyroSensor, 
      m_intakeBeamBreakSensor, 
      m_launcherBottomBeamBreakSensor, 
      m_launcherTopBeamBreakSensor, 
      m_intakeDistanceSensor, 
      m_launcherDistanceSensor, 
      m_driveSubsystem, 
      m_poseSubsystem,  
      m_intakeSubsystem, 
      m_launcherArmSubsystem, 
      m_launcherRollerSubsystem, 
      m_climberSubsystem, 
      m_lightsController
    );
    
    m_autoChooser = new SendableChooser<Command>();

    configureBindings();
    configureAutos();
  }

  private void configureBindings() {
    // DRIVER ========================================
    m_driveSubsystem.setDefaultCommand(m_driveSubsystem.driveWithControllerCommand(m_driverController::getLeftY, m_driverController::getLeftX, m_driverController::getRightX));
    m_driverController.leftTrigger().whileTrue(m_gameCommands.runIntakeCommand(IntakeLocation.Rear));
    m_driverController.rightTrigger().whileTrue(m_gameCommands.runIntakeCommand(IntakeLocation.Front));
    m_driverController.leftBumper().whileTrue(m_gameCommands.runEjectCommand(IntakeLocation.Rear));
    m_driverController.rightBumper().whileTrue(m_gameCommands.runEjectCommand(IntakeLocation.Front));
    m_driverController.leftStick().whileTrue(m_driveSubsystem.setLockedCommand());
    // m_driverController.rightStick().whileTrue(Commands.none());
    // m_driverController.povLeft().whileTrue(Commands.none());
    // m_driverController.povUp().whileTrue(Commands.none()); 
    // m_driverController.povRight().whileTrue(Commands.none());
    // m_driverController.povDown().whileTrue(Commands.none());
    m_driverController.a().whileTrue(m_gameCommands.alignRobotToTargetCommand()
      .andThen(Commands.runOnce(() -> m_gameCommands.rumbleControllers(new GameController[] {m_driverController})) // TODO: TEST
      ));
    m_driverController.b().whileTrue(m_gameCommands.moveToClimbCommand());
    m_driverController.y().whileTrue(m_gameCommands.climbCommand());
    // m_driverController.x().whileTrue(Commands.none());
    // m_driverController.start().whileTrue(Commands.none());
    m_driverController.back().onTrue(m_gyroSensor.resetCommand());

    // OPERATOR ========================================
    m_launcherArmSubsystem.setDefaultCommand(m_launcherArmSubsystem.alignManualCommand(m_operatorController::getLeftY));
    // TODO: if/when launcher auto angle alignment is working, make alignLauncherToTargetCommand the default command
    // m_operatorController.leftY().whileTrue(m_launcherArmSubsystem.alignManualCommand(m_operatorController::getLeftY));
    m_climberSubsystem.setDefaultCommand(m_climberSubsystem.moveArmManualCommand(m_operatorController::getRightY));
    m_operatorController.leftBumper().whileTrue(m_gameCommands.alignLauncherToAmpCommand(true)); // TODO: amp launcher position left trigger - position/launch or just position
    m_operatorController.rightTrigger().whileTrue(m_gameCommands.runLauncherCommand());
    m_operatorController.leftTrigger().whileTrue(m_gameCommands.runLauncherAmpCommand());
    //m_operatorController.rightBumper().whileTrue();
    // m_operatorController.leftStick().whileTrue(Commands.none());
    // m_operatorController.rightStick().whileTrue(Commands.none());
    m_operatorController.povLeft().whileTrue(m_gameCommands.alignLauncherToPositionCommand(Constants.Launcher.kArmPositionLongRange, true));
    m_operatorController.povUp().whileTrue(m_gameCommands.alignLauncherToPositionCommand(Constants.Launcher.kArmPositionMidRange, true));  
    m_operatorController.povRight().whileTrue(m_gameCommands.alignLauncherToPositionCommand(Constants.Launcher.kArmPositionShortRange, true));  
    m_operatorController.povDown().whileTrue(m_gameCommands.alignLauncherToPositionCommand(Constants.Launcher.kArmPositionSubwoofer, true));
    m_operatorController.a().whileTrue(m_gameCommands.alignLauncherToTargetCommand(true)); 
    m_operatorController.y().whileTrue(m_climberSubsystem.runRollersCommand(MotorDirection.Forward));
    m_operatorController.b().whileTrue(m_climberSubsystem.runRollersCommand(MotorDirection.Reverse));
    //m_operatorController.x().whileTrue(Commands.none());
    m_operatorController.start().whileTrue(m_launcherArmSubsystem.resetCommand());
    m_operatorController.back().whileTrue(m_gameCommands.resetSubsystems());

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

    SendableChooser<Double> intakePositionChooser = new SendableChooser<Double>();
    intakePositionChooser.setDefaultOption("" + Constants.Launcher.kArmPositionIntake, Constants.Launcher.kArmPositionIntake);
    intakePositionChooser.addOption("13.5", 13.5);
    intakePositionChooser.addOption("13.0", 13.0);
    intakePositionChooser.addOption("12.5", 12.5);
    intakePositionChooser.addOption("12.0", 12.0);
    intakePositionChooser.onChange(intakePosition -> m_launcherArmSubsystem.setIntakePosition(intakePosition));
    SmartDashboard.putData("Robot/Launcher/Arm/IntakePosition", intakePositionChooser);
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
    //m_autoChooser.addOption("BackupShoot1", m_autoCommands.runAuto(false, new AutoPoses[] { Constants.Game.Field.AutoWaypoints.kNotePreload1Poses })); // TODO: Make empty noteposes array
    m_autoChooser.addOption("ShootPickup1", m_autoCommands.scorePickup1());
    m_autoChooser.addOption("ShootPickup2", m_autoCommands.scorePickup2());
    m_autoChooser.addOption("ShootPickup3", m_autoCommands.scorePickup3());
    m_autoChooser.addOption("BackupShootPickup1", m_autoCommands.backupScorePickup1());
    m_autoChooser.addOption("BackupShootPickup14", m_autoCommands.backupScorePickup14());
    m_autoChooser.addOption("ScorePreload", m_autoCommands.scoreSubwooferAuto());
    
    SmartDashboard.putData("Robot/Auto/Command", m_autoChooser);
  }

  public Command getSelectedAutoCommand() {
    return m_autoChooser.getSelected();
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
    m_intakeBeamBreakSensor.updateTelemetry();
    m_launcherBottomBeamBreakSensor.updateTelemetry();
    m_launcherTopBeamBreakSensor.updateTelemetry();
    m_intakeDistanceSensor.updateTelemetry();
    m_launcherDistanceSensor.updateTelemetry();
    m_objectSensor.updateTelemetry();

    SmartDashboard.putNumber("Robot/Power/TotalCurrent", m_powerDistribution.getTotalCurrent());

    SmartDashboard.putBoolean(
      "Robot/HasInitialReset", 
      m_launcherArmSubsystem.hasInitialReset() &&
      m_climberSubsystem.hasInitialReset()
    );
  }
}
