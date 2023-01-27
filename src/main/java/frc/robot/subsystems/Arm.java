// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MercMath;




public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  TalonSRX arm;
  private static double NORMAL_P_VAL = 0.2;
  private static double NORMAL_I_VAL = 0.0;
  private static double NORMAL_D_VAL = 0.0;
  private static double NORMAL_F_VAL = 0.0;

  private final double NOMINAL_OUTPUT_FORWARD = 0.02;
  private final double NOMINAL_OUTPUT_REVERSE = -0.02;
  private final double PEAK_OUTPUT_FORWARD = 1.0;
  private final double PEAK_OUTPUT_REVERSE = -1.0;


  public Arm() {
    arm = new TalonSRX(9);
    
    arm.configFactoryDefault();

    arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.PID.PRIMARY_PID_LOOP, Constants.CTRE_TIMEOUT);
    arm.setSensorPhase(true);
    arm.setInverted(false);

    arm.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, Constants.CAN_STATUS_FREQ.HIGH);
    arm.configNominalOutputForward(NOMINAL_OUTPUT_FORWARD, Constants.CTRE_TIMEOUT);
    arm.configNominalOutputReverse(NOMINAL_OUTPUT_REVERSE, Constants.CTRE_TIMEOUT);
    arm.configPeakOutputForward(PEAK_OUTPUT_FORWARD, Constants.CTRE_TIMEOUT);
    arm.configPeakOutputReverse(PEAK_OUTPUT_REVERSE, Constants.CTRE_TIMEOUT);
    
    arm.configAllowableClosedloopError(Constants.PID.PRIMARY_PID_LOOP, 0, Constants.CTRE_TIMEOUT);
    
    arm.config_kP(Constants.PID.PRIMARY_PID_LOOP, NORMAL_P_VAL, Constants.CTRE_TIMEOUT);
    arm.config_kI(Constants.PID.PRIMARY_PID_LOOP, NORMAL_I_VAL, Constants.CTRE_TIMEOUT);
    arm.config_kD(Constants.PID.PRIMARY_PID_LOOP, NORMAL_D_VAL, Constants.CTRE_TIMEOUT);
    arm.config_kF(Constants.PID.PRIMARY_PID_LOOP, NORMAL_F_VAL, Constants.CTRE_TIMEOUT);

  }

  public void setPosition(double degrees) {
    double ticks = MercMath.degreesToEncoderTicks(degrees);
    arm.set(ControlMode.Position, ticks);
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
}
