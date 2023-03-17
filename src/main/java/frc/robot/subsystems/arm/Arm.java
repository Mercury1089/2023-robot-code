// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.SWERVE;
import frc.robot.util.MercMath;


public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  public static final int
    ARM_PID_SLOT = 0;

  private static final double 
    ARM_NORMAL_P_VAL = 1.0 / 23.0 * 1024.0,
    ARM_NORMAL_I_VAL = 0.0,
    ARM_NORMAL_D_VAL = 0.0,
    ARM_NORMAL_F_VAL = 0.0;

  private final double 
    NOMINAL_OUTPUT_FORWARD = 0.01, //0.02,
    PEAK_OUTPUT_FORWARD = 1.0, // 0.6,
    NOMINAL_OUTPUT_REVERSE = -0.01, //-0.5,
    PEAK_OUTPUT_REVERSE = -1.0;
  // Need to find upper and lower limit values
  public final double
    ARM_UPPER_LIMIT = 20,
    ARM_LOWER_LIMIT = -90;
  public final double GEAR_RATIO = 1;
  public final double THRESHOLD_DEGREES = 1.0;
  public final int SAFE_TO_TELE_POS = 11;

  private TalonFX arm;

  public Arm() {
    arm = new TalonFX(CAN.ARM_TALON);

    arm.configFactoryDefault();

    // Account for motor orientation.
    arm.setSensorPhase(true);
    arm.setInverted(true);

    SmartDashboard.putNumber("ARM P", ARM_NORMAL_P_VAL);
    SmartDashboard.putNumber("ARM I", ARM_NORMAL_I_VAL);
    SmartDashboard.putNumber("ARM D", ARM_NORMAL_D_VAL);
    SmartDashboard.putNumber("ARM MIN REV", NOMINAL_OUTPUT_REVERSE);
    SmartDashboard.putNumber("ARM MIN FWD", NOMINAL_OUTPUT_FORWARD);

    // arm.configForwardSoftLimitThreshold(MercMath.degreesToEncoderTicks(ARM_UPPER_LIMIT)*GEAR_RATIO, Constants.CTRE_TIMEOUT);
    // arm.configReverseSoftLimitThreshold(MercMath.degreesToEncoderTicks(ARM_LOWER_LIMIT)*GEAR_RATIO, Constants.CTRE_TIMEOUT);
    // arm.configForwardSoftLimitEnable(true, Constants.CTRE_TIMEOUT);
    // arm.configReverseSoftLimitEnable(true, Constants.CTRE_TIMEOUT);

    arm.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, ARM_PID_SLOT, Constants.CTRE.TIMEOUT_MS);
    arm.configSelectedFeedbackCoefficient(50 / 290000.0);

    arm.configForwardSoftLimitThreshold(47.0);
    arm.configForwardSoftLimitEnable(true);

    arm.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, Constants.CAN_STATUS_FREQ.HIGH);

    arm.configClearPositionOnLimitR(true, Constants.CTRE.TIMEOUT_MS);

    arm.configNominalOutputForward(NOMINAL_OUTPUT_FORWARD, Constants.CTRE.TIMEOUT_MS);
    arm.configNominalOutputReverse(NOMINAL_OUTPUT_REVERSE, Constants.CTRE.TIMEOUT_MS);
    arm.configPeakOutputForward(PEAK_OUTPUT_FORWARD, Constants.CTRE.TIMEOUT_MS);
    arm.configPeakOutputReverse(PEAK_OUTPUT_REVERSE, Constants.CTRE.TIMEOUT_MS);
    
    arm.configAllowableClosedloopError(ARM_PID_SLOT, 0, Constants.CTRE.TIMEOUT_MS);

    arm.config_kP(ARM_PID_SLOT, ARM_NORMAL_P_VAL, Constants.CTRE.TIMEOUT_MS);
    arm.config_kI(ARM_PID_SLOT, ARM_NORMAL_I_VAL, Constants.CTRE.TIMEOUT_MS);
    arm.config_kD(ARM_PID_SLOT, ARM_NORMAL_D_VAL, Constants.CTRE.TIMEOUT_MS);
    arm.config_kF(ARM_PID_SLOT, ARM_NORMAL_F_VAL, Constants.CTRE.TIMEOUT_MS);

    arm.selectProfileSlot(ARM_PID_SLOT, Constants.CTRE.PRIMARY_PID_LOOP);
  }

  public void configPID(double P, double I, double D, double nomFwd, double nomRev) {
    arm.config_kP(ARM_PID_SLOT, P, Constants.CTRE.TIMEOUT_MS);
    arm.config_kI(ARM_PID_SLOT, I, Constants.CTRE.TIMEOUT_MS);
    arm.config_kD(ARM_PID_SLOT, D, Constants.CTRE.TIMEOUT_MS);
    
    arm.configNominalOutputForward(nomFwd, Constants.CTRE.TIMEOUT_MS);
    arm.configNominalOutputReverse(nomRev, Constants.CTRE.TIMEOUT_MS);
  }

  /**  sets the position of the entire arm */
  public void setPosition(ArmPosition armPos) {
    arm.set(ControlMode.Position, armPos.degreePos);
    // arm.set(ControlMode.Position, (290000.0 / 50.0) * armPos.degreePos);
  }

  public void moveArm(Supplier<Double> speedSupplier) {
    arm.set(ControlMode.PercentOutput, 
      MathUtil.applyDeadband(speedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)
    );
  }

  public double getError() {
    return arm.getClosedLoopError(ARM_PID_SLOT);
  }

  public boolean isFinishedMoving() {
    return getError() < MercMath.degreesToEncoderTicks(THRESHOLD_DEGREES);
  }

  public double getArmPosition() {
    return arm.getSelectedSensorPosition();
  }

  public boolean isTelescopeSafe() {
    return getArmPosition() > SAFE_TO_TELE_POS;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("arm encoder", getArmPosition());
    SmartDashboard.putNumber("arm error", getError());
    SmartDashboard.putNumber("arm rev limit", arm.isRevLimitSwitchClosed());

    // configPID(
    //   SmartDashboard.getNumber("ARM P", ARM_LOWER_LIMIT),
    //   SmartDashboard.getNumber("ARM I", ARM_LOWER_LIMIT),
    //   SmartDashboard.getNumber("ARM D", ARM_LOWER_LIMIT),
    //   SmartDashboard.getNumber("ARM MIN REV", ARM_LOWER_LIMIT),
    //   SmartDashboard.getNumber("ARM MIN FWD", ARM_LOWER_LIMIT)
    // );

  }

  public enum ArmPosition {
    // enum values to be changed
    INSIDE(-2.0),
    FLOOR(0),
    RAMP_PICKUP(23.0),
    HIGH_SCORE(50.0),
    MID_SCORE(41.0),
    BULLDOZER(11.0),
    FELL_OVER(0); // lol

    public final double degreePos;

        ArmPosition(double degreePos) {
          this.degreePos = degreePos;
        }
  }

  
}
