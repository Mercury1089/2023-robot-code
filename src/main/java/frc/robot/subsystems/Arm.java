// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.util.MercMath;


public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  public static final int
    ARM_PID_SLOT = 1,
    TELESCOPE_PID_SLOT = 2,
    CLAW_PID_SLOT = 3;

  private static final double 
    ARM_NORMAL_P_VAL = 1.0,
    ARM_NORMAL_I_VAL = 0.0,
    ARM_NORMAL_D_VAL = 0.0,
    ARM_NORMAL_F_VAL = 0.0,
    TELESCOPE_NORMAL_P_VAL = 1.0,
    TELESCOPE_NORMAL_I_VAL = 0.0,
    TELESCOPE_NORMAL_D_VAL = 0.0,
    TELESCOPE_NORMAL_F_VAL = 0.0,
    CLAW_NORMAL_P_VAL = 1.0,
    CLAW_NORMAL_I_VAL = 0.0,
    CLAW_NORMAL_D_VAL = 0.0,
    CLAW_NORMAL_F_VAL = 0.0;

  public static final double
    SPROCKET_DIAMETER_INCHES = 6.0;

  private final double NOMINAL_OUTPUT_FORWARD = 0.02;
  private final double NOMINAL_OUTPUT_REVERSE = -0.02;
  private final double PEAK_OUTPUT_FORWARD = 1.0;
  private final double PEAK_OUTPUT_REVERSE = -1.0;
  // Need to find upper and lower limit values
  public final double
    ARM_UPPER_LIMIT = 20,
    ARM_LOWER_LIMIT = -90,
    CLAW_UPPER_LIMIT = 90,
    CLAW_LOWER_LIMIT = -90;
  // Need to find Gear ratio 30:1
  public final double GEAR_RATIO = 1;


  private TalonSRX arm, telescope, claw;
  private PigeonIMU pigeon;

  public Arm() {
    arm = new TalonSRX(CAN.ARM_TALON);
    telescope = new TalonSRX(CAN.TELESCOPE_TALON);
    claw = new TalonSRX(CAN.CLAW_TALON);
    // Configure Gyro
    pigeon = new PigeonIMU(CAN.ARM_GYRO);
    pigeon.configFactoryDefault();
    pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, Constants.CTRE_TIMEOUT);

    arm.configFactoryDefault();
    telescope.configFactoryDefault();
    claw.configFactoryDefault();

    // Account for motor orientation.
    arm.setInverted(false);
    telescope.setInverted(false);
    claw.setInverted(false);

    arm.setSensorPhase(false);
    telescope.setSensorPhase(false);
    claw.setSensorPhase(false);

    arm.configForwardSoftLimitThreshold(MercMath.degreesToEncoderTicks(ARM_UPPER_LIMIT)*GEAR_RATIO, Constants.CTRE_TIMEOUT);
    arm.configReverseSoftLimitThreshold(MercMath.degreesToEncoderTicks(ARM_LOWER_LIMIT)*GEAR_RATIO, Constants.CTRE_TIMEOUT);
    arm.configForwardSoftLimitEnable(true, Constants.CTRE_TIMEOUT);
    arm.configReverseSoftLimitEnable(true, Constants.CTRE_TIMEOUT);
    claw.configForwardSoftLimitThreshold(MercMath.degreesToEncoderTicks(CLAW_UPPER_LIMIT)*GEAR_RATIO, Constants.CTRE_TIMEOUT);
    claw.configReverseSoftLimitThreshold(MercMath.degreesToEncoderTicks(CLAW_LOWER_LIMIT)*GEAR_RATIO, Constants.CTRE_TIMEOUT);
    claw.configForwardSoftLimitEnable(true, Constants.CTRE_TIMEOUT);
    claw.configReverseSoftLimitEnable(true, Constants.CTRE_TIMEOUT);

    arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, ARM_PID_SLOT, Constants.CTRE_TIMEOUT);
    telescope.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, TELESCOPE_PID_SLOT, Constants.CTRE_TIMEOUT);
    claw.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, CLAW_PID_SLOT, Constants.CTRE_TIMEOUT);
    
    arm.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, Constants.CAN_STATUS_FREQ.HIGH);
    telescope.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, Constants.CAN_STATUS_FREQ.HIGH);
    claw.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, Constants.CAN_STATUS_FREQ.HIGH);

    arm.configNominalOutputForward(NOMINAL_OUTPUT_FORWARD, Constants.CTRE_TIMEOUT);
    arm.configNominalOutputReverse(NOMINAL_OUTPUT_REVERSE, Constants.CTRE_TIMEOUT);
    arm.configPeakOutputForward(PEAK_OUTPUT_FORWARD, Constants.CTRE_TIMEOUT);
    arm.configPeakOutputReverse(PEAK_OUTPUT_REVERSE, Constants.CTRE_TIMEOUT);
    telescope.configNominalOutputForward(NOMINAL_OUTPUT_FORWARD, Constants.CTRE_TIMEOUT);
    telescope.configNominalOutputReverse(NOMINAL_OUTPUT_REVERSE, Constants.CTRE_TIMEOUT);
    telescope.configPeakOutputForward(PEAK_OUTPUT_FORWARD, Constants.CTRE_TIMEOUT);
    telescope.configPeakOutputReverse(PEAK_OUTPUT_REVERSE, Constants.CTRE_TIMEOUT);
    claw.configNominalOutputForward(NOMINAL_OUTPUT_FORWARD, Constants.CTRE_TIMEOUT);
    claw.configNominalOutputReverse(NOMINAL_OUTPUT_REVERSE, Constants.CTRE_TIMEOUT);
    claw.configPeakOutputForward(PEAK_OUTPUT_FORWARD, Constants.CTRE_TIMEOUT);
    claw.configPeakOutputReverse(PEAK_OUTPUT_REVERSE, Constants.CTRE_TIMEOUT);
    
    arm.configAllowableClosedloopError(ARM_PID_SLOT, 0, Constants.CTRE_TIMEOUT);
    telescope.configAllowableClosedloopError(TELESCOPE_PID_SLOT, 0, Constants.CTRE_TIMEOUT);
    claw.configAllowableClosedloopError(CLAW_PID_SLOT, 0, Constants.CTRE_TIMEOUT);

    configPID(arm, ARM_PID_SLOT, ARM_NORMAL_P_VAL, ARM_NORMAL_I_VAL, ARM_NORMAL_D_VAL, ARM_NORMAL_F_VAL);
    configPID(telescope, TELESCOPE_PID_SLOT, TELESCOPE_NORMAL_P_VAL, TELESCOPE_NORMAL_I_VAL, TELESCOPE_NORMAL_D_VAL, TELESCOPE_NORMAL_F_VAL);
    configPID(claw, CLAW_PID_SLOT, CLAW_NORMAL_P_VAL, CLAW_NORMAL_I_VAL, CLAW_NORMAL_D_VAL, CLAW_NORMAL_F_VAL);

  }

  public void calibratePigeon() {
    pigeon.enterCalibrationMode(PigeonIMU.CalibrationMode.BootTareGyroAccel);
  }

  public void levelClaw() {
    double pitch = pigeon.getPitch();
    if (pitch != 0) {
      claw.set(ControlMode.Position, 0 - pitch);
    }
  }

  public void setPosition(double degrees) {
    double ticks = MercMath.degreesToEncoderTicks(degrees);
    arm.set(ControlMode.Position, ticks);
  }

  // sets the position of the entire arm
  public void setPosition(double armPos, double telePos, double clawPos) {
    arm.set(ControlMode.Position, MercMath.degreesToEncoderTicks(armPos));
    telescope.set(ControlMode.Position, MercMath.inchesToEncoderTicks(telePos));
    claw.set(ControlMode.Position, MercMath.degreesToEncoderTicks(clawPos));
  }

  public double getPosition() {
    return MercMath.encoderTicksToDegrees(arm.getSelectedSensorPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public enum ArmPosition {
    // enum values to be changed
    INSIDE(0),
    TOP_CONE(0),
    MID_CONE(0),
    TOP_CUBE(0),
    MID_CUBE(0),
    FLOOR(0),
    DOUBLE_SUBSTATION(0),
    FELL_OVER(0);

    public final double degreePos;

        ArmPosition(double degreePos) {
          this.degreePos = degreePos;
        }
  }

  public enum TelescopePosition {
    INSIDE(0),
    TOP_CONE(0),
    MID_CONE(0),
    TOP_CUBE(0),
    MID_CUBE(0),
    FLOOR(0),
    DOUBLE_SUBSTATION(0);

    public final double encPos;

    TelescopePosition(double encPos) {
      this.encPos = encPos;
    }
  }

  public enum ClawPosition {
    INSIDE(0),
    TOP_CONE(0),
    MID_CONE(0),
    TOP_CUBE(0),
    MID_CUBE(0),
    FLOOR(0),
    DOUBLE_SUBSTATION(0);

    public final double degreePos;

    ClawPosition(double degreePos) {
      this.degreePos = degreePos;
    }
  }

  private void configPID(TalonSRX talon, int slot, double p_val, double i_val, double d_val, double f_val) {
    talon.config_kP(slot, p_val, Constants.CTRE_TIMEOUT);
    talon.config_kI(slot, i_val, Constants.CTRE_TIMEOUT);
    talon.config_kD(slot, d_val, Constants.CTRE_TIMEOUT);
    talon.config_kF(slot, f_val, Constants.CTRE_TIMEOUT);
  }
}
