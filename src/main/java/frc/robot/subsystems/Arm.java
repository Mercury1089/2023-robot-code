// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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





  private final double NOMINAL_OUTPUT_FORWARD = 0.02;
  private final double NOMINAL_OUTPUT_REVERSE = -0.02;
  private final double PEAK_OUTPUT_FORWARD = 1.0;
  private final double PEAK_OUTPUT_REVERSE = -1.0;

  private TalonSRX arm, telescope, claw;
  private PigeonIMU clawPigeon;

  public Arm() {
    arm = new TalonSRX(9);
    telescope = new TalonSRX(0);
    claw = new TalonSRX(0);
    
    arm.configFactoryDefault();
    telescope.configFactoryDefault();
    claw.configFactoryDefault();

    // Account for motor orientation.
    arm.setInverted(false);
    telescope.setInverted(false);
    claw.setInverted(false);

    arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.PID.PRIMARY_PID_LOOP, Constants.CTRE_TIMEOUT);
    arm.setSensorPhase(true);

    arm.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, Constants.CAN_STATUS_FREQ.HIGH);

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
    
    arm.configAllowableClosedloopError(Constants.PID.PRIMARY_PID_LOOP, 0, Constants.CTRE_TIMEOUT);
    telescope.configAllowableClosedloopError(Constants.PID.PRIMARY_PID_LOOP, 0, Constants.CTRE_TIMEOUT);
    claw.configAllowableClosedloopError(Constants.PID.PRIMARY_PID_LOOP, 0, Constants.CTRE_TIMEOUT);

    configPID(arm, ARM_PID_SLOT, ARM_NORMAL_P_VAL, ARM_NORMAL_I_VAL, ARM_NORMAL_D_VAL, ARM_NORMAL_F_VAL);
    configPID(telescope, TELESCOPE_PID_SLOT, TELESCOPE_NORMAL_P_VAL, TELESCOPE_NORMAL_I_VAL, TELESCOPE_NORMAL_D_VAL, TELESCOPE_NORMAL_F_VAL);
    configPID(claw, CLAW_PID_SLOT, CLAW_NORMAL_P_VAL, CLAW_NORMAL_I_VAL, CLAW_NORMAL_D_VAL, CLAW_NORMAL_F_VAL);

  }

  public void setPosition(double degrees) {
    double ticks = MercMath.degreesToEncoderTicks(degrees);
    arm.set(ControlMode.Position, ticks);
  }

  public void setPosition(double armAngle, double armLength, double clawAngle) {
    double ticks = MercMath.degreesToEncoderTicks(armAngle);
    arm.set(ControlMode.Position, ticks);
    //telescope.set(ControlMode.Position, ticks);
    //claw.set(ControlMode.Position, ticks);
  }

  public double getPosition() {
    return MercMath.encoderTicksToDegrees(arm.getSelectedSensorPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public enum ArmPosition{
    // enums to be changed
    TOP(58000, false),       // Maximum height
    READY(43000, false),
    BOTTOM(-500, false),     // Negative value ensures we always move down until limit switch enabled
    HOOK(50000, false),      // Ready hook position
    HANG(-20000, true);    // Hang position - relative to current position.

    public final double encPos;
    public final boolean isRelative;

        /**
         * Creates an arm position, storing the encoder ticks
         * representing the height that the arm should be at.
         *
         * @param ep encoder position, in ticks
         */
        ArmPosition(double encPos, boolean isRelative) {
            this.encPos = encPos;
            this.isRelative = isRelative;
        }
  }

  private void configPID(TalonSRX talon, int slot, double p_val, double i_val, double d_val, double f_val) {
    talon.config_kP(slot, p_val, Constants.CTRE_TIMEOUT);
    talon.config_kI(slot, i_val, Constants.CTRE_TIMEOUT);
    talon.config_kD(slot, d_val, Constants.CTRE_TIMEOUT);
    talon.config_kF(slot, f_val, Constants.CTRE_TIMEOUT);
    
  }
}
