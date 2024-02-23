package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.common.Utils;
import frc.robot.lib.controllers.LightsController;
import frc.robot.lib.sensors.BeamBreakSensor;
import frc.robot.lib.sensors.DistanceSensor;
import frc.robot.lib.sensors.GyroSensor;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherArmSubsystem;
import frc.robot.subsystems.LauncherRollerSubsystem;
import frc.robot.subsystems.PoseSubsystem;

public class GameCommands {
  private final GyroSensor m_gyroSensor;   
  private final BeamBreakSensor m_intakeBeamBreakSensor;
  private final BeamBreakSensor m_launcherBottomBeamBreakSensor;
  private final BeamBreakSensor m_launcherTopBeamBreakSensor; 
  private final DistanceSensor m_intakeDistanceSensor;
  private final DistanceSensor m_launcherDistanceSensor;
  private final DriveSubsystem m_driveSubsystem;
  private final PoseSubsystem m_poseSubsystem;
  private final FeederSubsystem m_feederSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final LauncherArmSubsystem m_launcherArmSubsystem;
  private final LauncherRollerSubsystem m_launcherRollerSubsystem;
  private final ClimberSubsystem m_climberSubsystem;
  private final LightsController m_lightsController;

  public GameCommands(
    GyroSensor gyroSensor, 
    BeamBreakSensor intakeBeamBreakSensor,
    BeamBreakSensor launcherBottomBeamBreakSensor,
    BeamBreakSensor launcherTopBeamBreakSensor,
    DistanceSensor intakeDistanceSensor,
    DistanceSensor launcherDistanceSensor,
    DriveSubsystem driveSubsystem, 
    PoseSubsystem poseSubsystem,
    FeederSubsystem feederSubsystem,
    IntakeSubsystem intakeSubsystem,
    LauncherArmSubsystem launcherArmSubsystem,
    LauncherRollerSubsystem launcherRollerSubsystem,
    ClimberSubsystem climberSubsystem,
    LightsController lightsController
  ) {
    m_gyroSensor = gyroSensor;
    m_intakeBeamBreakSensor = intakeBeamBreakSensor;
    m_launcherBottomBeamBreakSensor = launcherBottomBeamBreakSensor;
    m_launcherTopBeamBreakSensor = launcherTopBeamBreakSensor;
    m_intakeDistanceSensor = intakeDistanceSensor;
    m_launcherDistanceSensor = launcherDistanceSensor;
    m_driveSubsystem = driveSubsystem;
    m_poseSubsystem = poseSubsystem;
    m_feederSubsystem = feederSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_launcherArmSubsystem = launcherArmSubsystem;
    m_launcherRollerSubsystem = launcherRollerSubsystem;
    m_climberSubsystem = climberSubsystem;
    m_lightsController = lightsController;
  }

 // TODO: [HIGHEST PRIORITY] refactor intake and launcher sensor logic for boolean beam breaks in place of distance calcuations

  public Command runFrontIntakeCommand() {
    return Commands.parallel(
      m_launcherArmSubsystem.alignToPositionCommand(Constants.Launcher.kArmPositionIntake),
      m_intakeSubsystem.runIntakeFromFrontCommand(m_intakeBeamBreakSensor::hasTarget, m_launcherBottomBeamBreakSensor::hasTarget)
        //.andThen(getNoteIntoLaunchPositionCommand(m_launcherDistanceSensor::getDistance)).withTimeout(5.0)
      )
    .withName("RunFrontIntakeCommand");
  }

  public Command runRearIntakeCommand() {
    return
    m_intakeSubsystem.runIntakeFromRearCommand(m_intakeBeamBreakSensor::hasTarget, m_launcherBottomBeamBreakSensor::hasTarget)
    .alongWith(m_launcherArmSubsystem.alignToPositionCommand(Constants.Launcher.kArmPositionIntake))
    // .andThen(getNoteIntoLaunchPositionCommand(m_launcherDistanceSensor::getDistance)).withTimeout(5.0)
    .withName("RunRearIntakeCommand");
  }

  // public Command getNoteIntoLaunchPositionCommand(Supplier<Double> distanceSupplier){
  //   return Commands.repeatingSequence(
  //     m_intakeSubsystem.runIntakeForLaunchPositionCommand().withTimeout(0.1) // Might need to slow down intake 0.2)
  //   )
  //   .until(() -> distanceSupplier.get() > 5.5) // Utils.isValueBetween(distanceSupplier.get(), 5.5, 10)
  //   .withName("getNoteIntoLaunchPosition " + distanceSupplier.get().toString());
  // }

  // TODO: make enum?
  public Command runEjectIntakeCommand(Boolean isRearEject) {
    if(isRearEject){
      return m_intakeSubsystem.runIntakeEjectRearCommand()
      .withName("RunEjectIntakeRearCommand");
    }
    return m_intakeSubsystem.runIntakeEjectFrontCommand()
      .withName("RunEjectIntakeFrontCommand");
    
  }

  // TODO: this needs more testing once vision cameras are mounted and configured
  public Command alignRobotToTargetCommand() {
    return
    m_driveSubsystem.alignToTargetCommand(m_poseSubsystem::getPose, getCurrentTargetPose())
    .withName("AlignRobotToTarget");
  }

  // TODO: this needs more testing once vision cameras are mounted and configured
  public Command alignLauncherToTargetCommand() {
    return
    m_launcherArmSubsystem.alignToTargetCommand(m_poseSubsystem::getPose, getCurrentTargetPose())
    .withName("AlignLauncherToTarget");
  }

  // TODO: refactor this command to use different speed configurations based on speaker or amp target using the isCurrentTargetAmp check
  public Command runLauncherCommand() {
    return Commands.parallel(
      m_launcherRollerSubsystem.runRollersCommand(-0.8, 0.8), //-0.4, 0.6
      Commands.sequence(
        new WaitCommand(2.0),
        m_intakeSubsystem.runIntakeForLaunchCommand()
        )
    )
    .onlyIf(() -> m_launcherBottomBeamBreakSensor.hasTarget())
    .withName("RunLauncher");
  }

  public Command moveToClimbCommand() {
    return Commands.parallel(
      m_launcherArmSubsystem.alignToPositionCommand(1.0),
        m_feederSubsystem.moveFeedOutCommand().withTimeout(0.5),
      m_climberSubsystem.moveArmToPositionCommand(Constants.Climber.kArmMotorForwardSoftLimit-0.1)
    )
    .withName("MoveToClimb");
  }

  public Command climbCommand() {
    return m_climberSubsystem.moveArmToPositionCommand(0.0) // TODO: update?
      .withName("Climb");
  }

  // TODO: work with build to confirm that hard stops for all arm mechanisms are in place so that zeroing out all in parallel is safe
  public Command resetSubsystems() {
    return 
    m_feederSubsystem.resetCommand()
    .alongWith(
      m_launcherArmSubsystem.resetCommand(),
      m_climberSubsystem.resetCommand()
    )
    .withName("ResetSubsystems");
  }

  // TODO: update values based on testing: +/- 10 degrees rotation to amp and within 1 meter
  private boolean isCurrentTargetAmp(Pose2d robotPose) {
    Pose2d targetPose = getAmpPose().toPose2d();
    return 
    robotPose.getRotation().minus(targetPose.getRotation()).getDegrees() <= 10.0 && 
    robotPose.getTranslation().getDistance(targetPose.getTranslation()) < 1.0;
  }

  private Pose3d getCurrentTargetPose() {
    boolean isCurrTargetAmp = isCurrentTargetAmp(m_poseSubsystem.getPose());
    SmartDashboard.putBoolean("currentTargetIsAmp", isCurrTargetAmp);
    return isCurrentTargetAmp(m_poseSubsystem.getPose()) ? getAmpPose() : getSpeakerPose();

  }


  private Pose3d getSpeakerPose() {
    return Robot.getAlliance() == Alliance.Blue ? Constants.Game.Field.Targets.kBlueSpeaker : Constants.Game.Field.Targets.kRedSpeaker;
  }

  private Pose3d getAmpPose() {
    return Robot.getAlliance() == Alliance.Blue ? Constants.Game.Field.Targets.kBlueAmp : Constants.Game.Field.Targets.kRedAmp;
  }
}
