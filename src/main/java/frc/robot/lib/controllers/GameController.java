package frc.robot.lib.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

  @Override
  public double getLeftX() {
    return Utils.squareInput(-super.getLeftX(), Constants.Controllers.kInputDeadband);
  }

  @Override
  public double getRightX() {
    return Utils.squareInput(-super.getRightX(), Constants.Controllers.kInputDeadband);
  }

  @Override
  public double getRightY() {
    return Utils.squareInput(-super.getRightY(), Constants.Controllers.kInputDeadband);
  }

  // TODO: implement any desired rumble pattern commands as extensions
}
