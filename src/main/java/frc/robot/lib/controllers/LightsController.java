package frc.robot.lib.controllers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.common.Enums.LightsMode;

public class LightsController {

  public LightsController() {
    SmartDashboard.putString("Robot/Lights/Mode", LightsMode.Default.toString().toUpperCase());
  }

  public void setLightsMode(LightsMode lightsMode) {
    SmartDashboard.putString("Robot/Lights/Mode", lightsMode.toString().toUpperCase());
  }

}
