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
import frc.robot.subsystems.GamePieceLEDs;
import frc.robot.subsystems.GamePieceLEDs.GamePiece;
import frc.robot.util.MercMath;

public class Claw extends SubsystemBase {

  public static final int CLAW_PID_SLOT = 2;

  private static final double
    CLAW_NORMAL_P_VAL = 1.0,
    CLAW_NORMAL_I_VAL = 0.0,
    CLAW_NORMAL_D_VAL = 0.0,
    CLAW_NORMAL_F_VAL = 0.0;

  private final double 
    NOMINAL_OUTPUT_FORWARD = 0.02,
    NOMINAL_OUTPUT_REVERSE = -0.02,
    PEAK_OUTPUT_FORWARD = 1,
    PEAK_OUTPUT_REVERSE = -1;

  private TalonFX claw;
  /** Creates a new Claw. */
  public Claw() {

    claw = new TalonFX(CAN.CLAW_TALON);

    claw.configFactoryDefault();

    claw.setSensorPhase(true);
    claw.setInverted(true);
    

    claw.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, CLAW_PID_SLOT, Constants.CTRE_TIMEOUT);

    claw.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, Constants.CAN_STATUS_FREQ.HIGH);

    claw.configNominalOutputForward(NOMINAL_OUTPUT_FORWARD, Constants.CTRE_TIMEOUT);
    claw.configNominalOutputReverse(NOMINAL_OUTPUT_REVERSE, Constants.CTRE_TIMEOUT);
    claw.configPeakOutputForward(PEAK_OUTPUT_FORWARD, Constants.CTRE_TIMEOUT);
    claw.configPeakOutputReverse(PEAK_OUTPUT_REVERSE, Constants.CTRE_TIMEOUT);

    claw.configAllowableClosedloopError(CLAW_PID_SLOT, 0, Constants.CTRE_TIMEOUT);

    claw.config_kP(CLAW_PID_SLOT, CLAW_NORMAL_P_VAL, Constants.CTRE_TIMEOUT);
    claw.config_kI(CLAW_PID_SLOT, CLAW_NORMAL_I_VAL, Constants.CTRE_TIMEOUT);
    claw.config_kD(CLAW_PID_SLOT, CLAW_NORMAL_D_VAL, Constants.CTRE_TIMEOUT);
    claw.config_kF(CLAW_PID_SLOT, CLAW_NORMAL_F_VAL, Constants.CTRE_TIMEOUT);

  }

  public void adjustClawMode(GamePieceLEDs leds) {
    GamePiece gamePiece = leds.getGameState();
    if (gamePiece == GamePiece.CONE) {
      setClawPosition(ClawPosition.CONE);
    } else if (gamePiece == GamePiece.CUBE) {
      setClawPosition(ClawPosition.CUBE);
    } else {
      setClawPosition(ClawPosition.NONE); 
    }
  }

  public void moveClaw(Supplier<Double> speedSupplier) {
    claw.set(ControlMode.PercentOutput, -speedSupplier.get() * 0.25);
  }

  public void setClawPosition(ClawPosition position) {
    claw.set(ControlMode.Position, MercMath.encoderTicksToDegrees(position.encoderPosition));
  }

  public enum ClawPosition {
    NONE(0),
    CONE(0),
    CUBE(0);

    public final double encoderPosition;

    private ClawPosition(double encoderPos) {
      this.encoderPosition = encoderPos;

    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Claw Position", claw.getSelectedSensorPosition());
  }
}