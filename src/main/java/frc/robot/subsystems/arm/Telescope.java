// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.util.MercMath;

public class Telescope extends SubsystemBase {
  /** Creates a new Telescope. */
  public static final int TELESCOPE_PID_SLOT = 0;

  private static final double
    TELESCOPE_NORMAL_P_VAL = 1.0,
    TELESCOPE_NORMAL_I_VAL = 0.0,
    TELESCOPE_NORMAL_D_VAL = 0.0,
    TELESCOPE_NORMAL_F_VAL = 0.0;

  public static final double
    SPROCKET_DIAMETER_INCHES = 1.5;

  // Gear Ratio 30:1
  public final double GEAR_RATIO = 30.0;

  private final double 
    NOMINAL_OUTPUT_FORWARD = 0.02,
    NOMINAL_OUTPUT_REVERSE = -0.02,
    PEAK_OUTPUT_FORWARD = 1,
    PEAK_OUTPUT_REVERSE = -1;

  public final double THRESHOLD_INCHES = 1.0;

  private TalonFX telescope;

  public Telescope() {
    telescope = new TalonFX(CAN.TELESCOPE_TALON);

    telescope.configFactoryDefault();

    telescope.setInverted(false);
    telescope.setSensorPhase(false);

    telescope.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, TELESCOPE_PID_SLOT, Constants.CTRE_TIMEOUT);
    telescope.configSelectedFeedbackCoefficient((SPROCKET_DIAMETER_INCHES * Math.PI) / (GEAR_RATIO * Constants.UNITS.FALCON_ENCODER_TICKS_PER_REVOLUTION), TELESCOPE_PID_SLOT, Constants.CTRE_TIMEOUT);

    telescope.configForwardSoftLimitThreshold(13.0);
    telescope.configForwardSoftLimitEnable(true);
    
    telescope.configClearPositionOnLimitR(true, Constants.CTRE_TIMEOUT);
    
    
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

  public void moveTelescope(Supplier<Double> speedSupplier) {
    telescope.set(ControlMode.PercentOutput, speedSupplier.get() * 0.5);
  }

  public void setPosition(TelescopePosition telePos) {
    telescope.set(ControlMode.Position, MercMath.inchesToEncoderTicks(telePos.encPos));
  }

  public double getError() {
    return telescope.getClosedLoopError(TELESCOPE_PID_SLOT);
  }

  public boolean isFinishedMoving() {
    return getError() < MercMath.inchesToEncoderTicks(THRESHOLD_INCHES);
  }
  
  public double getTelescopePosition() {
    return telescope.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("telescope encoder", telescope.getSelectedSensorPosition(TELESCOPE_PID_SLOT));
    SmartDashboard.putNumber("telescope error", getError());
    SmartDashboard.putNumber("Foward limit switch", telescope.isFwdLimitSwitchClosed());
    SmartDashboard.putNumber("Reverse limit switch", telescope.isRevLimitSwitchClosed());
  }

  public enum TelescopePosition {
    INSIDE(0),
    OUT(0),
    PICK_UP(0),
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
