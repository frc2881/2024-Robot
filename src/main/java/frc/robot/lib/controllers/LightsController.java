package frc.robot.lib.controllers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.common.Enums.LightsMode;

// TODO: update Python script on coprocessor side to use updated modes/enum values

public class LightsController {
  public LightsController() {
    SmartDashboard.putString("Robot/Lights/Mode", LightsMode.Default.toString());
  }

  public void setLightsMode(LightsMode lightsMode) {
    SmartDashboard.putString("Robot/Lights/Mode", lightsMode.toString());
  }
}
