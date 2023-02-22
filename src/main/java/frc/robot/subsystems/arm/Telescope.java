// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.util.MercMath;

public class Telescope extends SubsystemBase {
  /** Creates a new Telescope. */
  public static final int TELESCOPE_PID_SLOT = 2;

  private static final double
    TELESCOPE_NORMAL_P_VAL = 1.0,
    TELESCOPE_NORMAL_I_VAL = 0.0,
    TELESCOPE_NORMAL_D_VAL = 0.0,
    TELESCOPE_NORMAL_F_VAL = 0.0;

  public static final double
    SPROCKET_DIAMETER_INCHES = 6.0;

  private final double 
    NOMINAL_OUTPUT_FORWARD = 0.02,
    NOMINAL_OUTPUT_REVERSE = -0.02,
    PEAK_OUTPUT_FORWARD = 0.2,
    PEAK_OUTPUT_REVERSE = -0.2;

  private TalonSRX telescope;

  public Telescope() {
    telescope = new TalonSRX(CAN.TELESCOPE_TALON);

    telescope.configFactoryDefault();

    telescope.setInverted(false);
    telescope.setSensorPhase(false);

    telescope.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, TELESCOPE_PID_SLOT, Constants.CTRE_TIMEOUT);

    telescope.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, Constants.CAN_STATUS_FREQ.HIGH);

    telescope.configNominalOutputForward(NOMINAL_OUTPUT_FORWARD, Constants.CTRE_TIMEOUT);
    telescope.configNominalOutputReverse(NOMINAL_OUTPUT_REVERSE, Constants.CTRE_TIMEOUT);
    telescope.configPeakOutputForward(PEAK_OUTPUT_FORWARD, Constants.CTRE_TIMEOUT);
    telescope.configPeakOutputReverse(PEAK_OUTPUT_REVERSE, Constants.CTRE_TIMEOUT);

    telescope.configAllowableClosedloopError(TELESCOPE_PID_SLOT, 0, Constants.CTRE_TIMEOUT);

    telescope.config_kP(TELESCOPE_PID_SLOT, TELESCOPE_NORMAL_P_VAL, Constants.CTRE_TIMEOUT);
    telescope.config_kI(TELESCOPE_PID_SLOT, TELESCOPE_NORMAL_I_VAL, Constants.CTRE_TIMEOUT);
    telescope.config_kD(TELESCOPE_PID_SLOT, TELESCOPE_NORMAL_D_VAL, Constants.CTRE_TIMEOUT);
    telescope.config_kF(TELESCOPE_PID_SLOT, TELESCOPE_NORMAL_F_VAL, Constants.CTRE_TIMEOUT);
  }

  public void setTelescopePosition(TelescopePosition telePos) {
    telescope.set(ControlMode.Position, MercMath.inchesToEncoderTicks(telePos.encPos));
  }
  
  public double getTelescopePosition() {
    return telescope.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public enum TelescopePosition {
    INSIDE(0),
    TOP_CONE(0),
    MID_CONE(0),
    TOP_CUBE(0),
    MID_CUBE(0),
    FLOOR(0),
    DOUBLE_SUBSTATION(0),
    FELL_OVER(0);

    public final double encPos;

    TelescopePosition(double encPos) {
      this.encPos = encPos;
    }
  }
}
