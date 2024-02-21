package frc.robot.lib.sensors;

import com.revrobotics.jni.VL53L0XJNI;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;

public class VL53L0X {
    private int m_addr = 0x53;
    private int m_port;

    public VL53L0X(Port port) {
      m_port = (port == Port.kOnboard) ? 0 : 1;
      VL53L0XJNI.Init(m_port, m_addr);
      if (!initialize()) {
        DriverStation.reportError(String.format("Error initializing device on port %s. Please check your connections", port == Port.kMXP ? "MXP" : "Onboard"), false);
      }
    }

    private synchronized boolean initialize() {
        boolean status = false;
        int statusInt;
        if ((statusInt = VL53L0XJNI.ValidateI2C(m_port, m_addr)) != 0) {
          DriverStation.reportError(String.format("Error 0x%08X: Could not communicate with sensor over I2C.", statusInt), false);
        }
        status = VL53L0XJNI.DataInit(m_port, m_addr);
        if (status)
            status = VL53L0XJNI.GetDeviceInfo(m_port, m_addr);
        if (status)
            status = VL53L0XJNI.StaticInit(m_port, m_addr);
        if (status)
            status = VL53L0XJNI.PerformRefCalibration(m_port, m_addr);
        if (status)
            status = VL53L0XJNI.PerformRefSpadManagement(m_port, m_addr);
        if (status)
            status = VL53L0XJNI.SetDeviceMode(1, m_port, m_addr);
        if (status) {
          status = setRangeProfileDefault();
        }
        return status;
    }

    

    public void startRanging() {
      VL53L0XJNI.StartMeasurement(m_port, m_addr);
    }

    public double getDistance() {
      double distance = -1.0;
      if (VL53L0XJNI.GetMeasurementDataReady(m_port, m_addr)) {
        distance = VL53L0XJNI.GetRangingMeasurementData(m_port, m_addr);
        VL53L0XJNI.ClearInterruptMask(m_port, m_addr);
      }
      return distance > 0 ? distance : -1.0;
    }

    private boolean setRangeProfileDefault() {
        boolean status = false;
        status = VL53L0XJNI.SetLimitCheckEnableSigmaFinalRange(1, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetLimitCheckEnableSignalRateFinalRange(1, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetLimitCheckEnableRangeIgnoreThreshold(0, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetLimitCheckValueSigmaFinalRange(18, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetLimitCheckValueSignalRateFinalRange(0.25, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetMeasurementTimingBudgetMicroSeconds(33823, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetVcselPulsePeriodPreRange(14, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetVcselPulsePeriodFinalRange(10, m_port, m_addr);
        return status;
    }

    private boolean setRangeProfileLongRange() {
        boolean status = false;
        status = VL53L0XJNI.SetLimitCheckEnableSigmaFinalRange(1, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetLimitCheckEnableSignalRateFinalRange(1, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetLimitCheckEnableRangeIgnoreThreshold(0, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetLimitCheckValueSigmaFinalRange(60, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetLimitCheckValueSignalRateFinalRange(0.1, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetMeasurementTimingBudgetMicroSeconds(33000, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetVcselPulsePeriodPreRange(18, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetVcselPulsePeriodFinalRange(14, m_port, m_addr);
        return status;
    }

    private boolean setRangeProfileHighAccuracy() {
        boolean status = false;
        status = VL53L0XJNI.SetLimitCheckEnableSigmaFinalRange(1, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetLimitCheckEnableSignalRateFinalRange(1, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetLimitCheckEnableRangeIgnoreThreshold(0, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetLimitCheckValueSigmaFinalRange(18, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetLimitCheckValueSignalRateFinalRange(0.25, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetMeasurementTimingBudgetMicroSeconds(200000, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetVcselPulsePeriodPreRange(14, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetVcselPulsePeriodFinalRange(10, m_port, m_addr);
        return status;
    }

    private boolean setRangeProfileHighSpeed() {
        boolean status = false;
        status = VL53L0XJNI.SetLimitCheckEnableSigmaFinalRange(1, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetLimitCheckEnableSignalRateFinalRange(1, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetLimitCheckEnableRangeIgnoreThreshold(0, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetLimitCheckValueSigmaFinalRange(32, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetLimitCheckValueSignalRateFinalRange(0.25, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetMeasurementTimingBudgetMicroSeconds(30000, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetVcselPulsePeriodPreRange(14, m_port, m_addr);
        if(status)
            status = VL53L0XJNI.SetVcselPulsePeriodFinalRange(10, m_port, m_addr);
        return status;
    }


}
