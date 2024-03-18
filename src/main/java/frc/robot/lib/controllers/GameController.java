package frc.robot.lib.controllers;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.lib.common.Utils;

public class GameController extends CommandXboxController {

  public GameController(int port) {
    super(port);
  }

  @Override
  public double getLeftY() {
    return Utils.squareInput(-super.getLeftY(), Constants.Controllers.kInputDeadband);
  }

  public Trigger leftY() {
    return new Trigger(() -> Math.abs(super.getLeftY()) > Constants.Controllers.kInputDeadband);
  }

  @Override
  public double getLeftX() {
    return Utils.squareInput(-super.getLeftX(), Constants.Controllers.kInputDeadband);
  }

  public Trigger leftX() {
    return new Trigger(() -> Math.abs(super.getLeftX()) > Constants.Controllers.kInputDeadband);
  }

  @Override
  public double getRightX() {
    return Utils.squareInput(-super.getRightX(), Constants.Controllers.kInputDeadband);
  }

  public Trigger rightX() {
    return new Trigger(() -> Math.abs(super.getRightX()) > Constants.Controllers.kInputDeadband);
  }

  @Override
  public double getRightY() {
    return Utils.squareInput(-super.getRightY(), Constants.Controllers.kInputDeadband);
  }

  public Trigger rightY() {
    return new Trigger(() -> Math.abs(super.getRightY()) > Constants.Controllers.kInputDeadband);
  }

  public Command rumbleShort() {
    return Commands.startEnd(
      () -> super.getHID().setRumble(RumbleType.kBothRumble, 1), 
      () -> super.getHID().setRumble(RumbleType.kBothRumble, 0));
  }
}
