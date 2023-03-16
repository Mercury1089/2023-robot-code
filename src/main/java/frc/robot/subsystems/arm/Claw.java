// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.subsystems.GamePieceLEDs;
import frc.robot.subsystems.GamePieceLEDs.GamePiece;

public class Claw extends SubsystemBase {

  public static final int CLAW_PID_SLOT = 0;

  private static final double
    CLAW_NORMAL_P_VAL = 1.0 / 25.0 * 1024.0,
    CLAW_NORMAL_I_VAL = 0.0,
    CLAW_NORMAL_D_VAL = 0.0,
    CLAW_NORMAL_F_VAL = 0.0;

  private final double 
    NOMINAL_OUTPUT_FORWARD = 0.02,
    NOMINAL_OUTPUT_REVERSE = -0.02,
    PEAK_OUTPUT_FORWARD = 1,
    PEAK_OUTPUT_REVERSE = -1;

  public static final double
    SPROCKET_DIAMETER_INCHES = 1.5;
    
  private TalonSRX claw;
  /** Creates a new Claw. */
  public Claw() {
    claw = new TalonSRX(CAN.CLAW_TALON);

    claw.configFactoryDefault();

    claw.setSensorPhase(true);
    claw.setInverted(false);
    
    claw.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, CLAW_PID_SLOT, Constants.CTRE.TIMEOUT_MS);
    // claw.configSelectedFeedbackCoefficient(2*(SPROCKET_DIAMETER_INCHES * Math.PI) / Constants.UNITS.MAG_ENCODER_TICKS_PER_REVOLUTION, CLAW_PID_SLOT, Constants.CTRE_TIMEOUT);
    claw.configSelectedFeedbackCoefficient(100/6200.0, CLAW_PID_SLOT, Constants.CTRE.TIMEOUT_MS);
    claw.configForwardSoftLimitThreshold(99.0);
    claw.configForwardSoftLimitEnable(true, Constants.CTRE.TIMEOUT_MS);

    claw.configClearPositionOnLimitR(true, Constants.CTRE.TIMEOUT_MS);
    claw.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, Constants.CAN_STATUS_FREQ.HIGH);

    claw.configNominalOutputForward(NOMINAL_OUTPUT_FORWARD, Constants.CTRE.TIMEOUT_MS);
    claw.configNominalOutputReverse(NOMINAL_OUTPUT_REVERSE, Constants.CTRE.TIMEOUT_MS);
    claw.configPeakOutputForward(PEAK_OUTPUT_FORWARD, Constants.CTRE.TIMEOUT_MS);
    claw.configPeakOutputReverse(PEAK_OUTPUT_REVERSE, Constants.CTRE.TIMEOUT_MS);

    claw.configAllowableClosedloopError(CLAW_PID_SLOT, 0, Constants.CTRE.TIMEOUT_MS);

    claw.config_kP(CLAW_PID_SLOT, CLAW_NORMAL_P_VAL, Constants.CTRE.TIMEOUT_MS);
    claw.config_kI(CLAW_PID_SLOT, CLAW_NORMAL_I_VAL, Constants.CTRE.TIMEOUT_MS);
    claw.config_kD(CLAW_PID_SLOT, CLAW_NORMAL_D_VAL, Constants.CTRE.TIMEOUT_MS);
    claw.config_kF(CLAW_PID_SLOT, CLAW_NORMAL_F_VAL, Constants.CTRE.TIMEOUT_MS);

    claw.selectProfileSlot(CLAW_PID_SLOT, Constants.CTRE.PRIMARY_PID_LOOP);
  }

  public void close(GamePieceLEDs leds) {
    GamePiece gamePiece = leds.getGameState();
    if (gamePiece == GamePiece.CONE) {
      setClawPosition(ClawPosition.CONE);
    } else if (gamePiece == GamePiece.CUBE) {
      setClawPosition(ClawPosition.CUBE);
    } else {
      setClawPosition(ClawPosition.OPEN);
    }
  }

  public void open() {
    setClawPosition(ClawPosition.OPEN);
  }


  public void moveClaw(Supplier<Double> speedSupplier) {
    claw.set(ControlMode.PercentOutput, speedSupplier.get() * 0.25);
  }

  public void setClawPosition(ClawPosition position) {
    claw.set(ControlMode.Position, position.position);
  }
  

  public enum ClawPosition {
    OPEN(0),
    CONE(87),
    CUBE(54);

    public final double position;

    private ClawPosition(double pos) {
      this.position = pos;
    }
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Claw Position", claw.getSelectedSensorPosition(CLAW_PID_SLOT));
    SmartDashboard.putNumber("claw fwd limit", claw.isFwdLimitSwitchClosed()); // yellow
    SmartDashboard.putNumber("claw reverse limit", claw.isRevLimitSwitchClosed());
    SmartDashboard.putNumber("claw supply current", claw.getSupplyCurrent());
  }
}