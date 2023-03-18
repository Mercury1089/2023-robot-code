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
import frc.robot.subsystems.arm.Arm;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.util.MercMath;

public class Telescope extends SubsystemBase {
  /** Creates a new Telescope. */
  public static final int TELESCOPE_PID_SLOT = 0;

  private static final double
    TELESCOPE_NORMAL_P_VAL = 1.0 / 7.0 * 1024.0,
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
    PEAK_OUTPUT_FORWARD = 0.55,
    PEAK_OUTPUT_REVERSE = -1;

  public final double THRESHOLD_INCHES = 1.0;

  private TalonFX telescope;

  public Telescope() {
    telescope = new TalonFX(CAN.TELESCOPE_TALON);

    telescope.configFactoryDefault();

    telescope.setInverted(false);
    telescope.setSensorPhase(false);

    telescope.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, TELESCOPE_PID_SLOT, Constants.CTRE.TIMEOUT_MS);
    telescope.configSelectedFeedbackCoefficient((SPROCKET_DIAMETER_INCHES * Math.PI) / (GEAR_RATIO * Constants.UNITS.FALCON_ENCODER_TICKS_PER_REVOLUTION),
    TELESCOPE_PID_SLOT, Constants.CTRE.TIMEOUT_MS);

    telescope.configForwardSoftLimitThreshold(13.0);
    telescope.configForwardSoftLimitEnable(true);
    
    telescope.configClearPositionOnLimitR(true, Constants.CTRE.TIMEOUT_MS);
  
    telescope.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, Constants.CAN_STATUS_FREQ.HIGH);

    telescope.configNominalOutputForward(NOMINAL_OUTPUT_FORWARD, Constants.CTRE.TIMEOUT_MS);
    telescope.configNominalOutputReverse(NOMINAL_OUTPUT_REVERSE, Constants.CTRE.TIMEOUT_MS);
    telescope.configPeakOutputForward(PEAK_OUTPUT_FORWARD, Constants.CTRE.TIMEOUT_MS);
    telescope.configPeakOutputReverse(PEAK_OUTPUT_REVERSE, Constants.CTRE.TIMEOUT_MS);

    telescope.configAllowableClosedloopError(TELESCOPE_PID_SLOT, 0, Constants.CTRE.TIMEOUT_MS);

    telescope.config_kP(TELESCOPE_PID_SLOT, TELESCOPE_NORMAL_P_VAL, Constants.CTRE.TIMEOUT_MS);
    telescope.config_kI(TELESCOPE_PID_SLOT, TELESCOPE_NORMAL_I_VAL, Constants.CTRE.TIMEOUT_MS);
    telescope.config_kD(TELESCOPE_PID_SLOT, TELESCOPE_NORMAL_D_VAL, Constants.CTRE.TIMEOUT_MS);
    telescope.config_kF(TELESCOPE_PID_SLOT, TELESCOPE_NORMAL_F_VAL, Constants.CTRE.TIMEOUT_MS);

    telescope.selectProfileSlot(TELESCOPE_PID_SLOT, Constants.CTRE.PRIMARY_PID_LOOP);
  }

  public void moveTelescope(Supplier<Double> speedSupplier) {
    telescope.set(ControlMode.PercentOutput, speedSupplier.get() * 0.5);
  }

  public void setPosition(TelescopePosition telePos) {
    telescope.set(ControlMode.Position, telePos.encPos);
    
  }

  public double getError() {
    return telescope.getClosedLoopError(TELESCOPE_PID_SLOT);
  }

  public boolean isFinishedMoving() {
    return getError() < THRESHOLD_INCHES;
  }
  
  public double getTelescopePosition() {
    return telescope.getSelectedSensorPosition();
  }

  public boolean isAtPosition(TelescopePosition position) {
    return Math.abs(getTelescopePosition() - position.encPos) < THRESHOLD_INCHES;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("telescope encoder", telescope.getSelectedSensorPosition(TELESCOPE_PID_SLOT));
    SmartDashboard.putNumber("telescope error", getError());
    SmartDashboard.putNumber("Foward limit switch", telescope.isFwdLimitSwitchClosed());
    SmartDashboard.putNumber("Reverse limit switch", telescope.isRevLimitSwitchClosed());

    SmartDashboard.putBoolean("telescope isAtPosition", isAtPosition(TelescopePosition.INSIDE));
  }

  public enum TelescopePosition {
    HOME(-3.0),
    INSIDE(0.0),
    FLOOR(0),
    RAMP_PICKUP(0.0),
    HIGH_SCORE(13.0),
    MID_SCORE(0.0),
    BULLDOZER(12),
    FELL_OVER(0); // lol

    public final double encPos;

    TelescopePosition(double encPos) {
      this.encPos = encPos;
    }
  }
}
