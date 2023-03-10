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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.util.MercMath;


public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  public static final int
    ARM_PID_SLOT = 1;

  private static final double 
    ARM_NORMAL_P_VAL = 1.0,
    ARM_NORMAL_I_VAL = 0.0,
    ARM_NORMAL_D_VAL = 0.0,
    ARM_NORMAL_F_VAL = 0.0;

  private final double 
    NOMINAL_OUTPUT_FORWARD = 0.02,
    NOMINAL_OUTPUT_REVERSE = -0.02,
    PEAK_OUTPUT_FORWARD = 1.0,
    PEAK_OUTPUT_REVERSE = -1.0;
  // Need to find upper and lower limit values
  public final double
    ARM_UPPER_LIMIT = 20,
    ARM_LOWER_LIMIT = -90;
  public final double GEAR_RATIO = 1;
  public final double THRESHOLD_DEGREES = 1.0;

  private TalonFX arm;

  public Arm() {
    arm = new TalonFX(CAN.ARM_TALON);

    arm.configFactoryDefault();

    // Account for motor orientation.
    arm.setSensorPhase(true);
    arm.setInverted(true);
    

    // arm.configForwardSoftLimitThreshold(MercMath.degreesToEncoderTicks(ARM_UPPER_LIMIT)*GEAR_RATIO, Constants.CTRE_TIMEOUT);
    // arm.configReverseSoftLimitThreshold(MercMath.degreesToEncoderTicks(ARM_LOWER_LIMIT)*GEAR_RATIO, Constants.CTRE_TIMEOUT);
    // arm.configForwardSoftLimitEnable(true, Constants.CTRE_TIMEOUT);
    // arm.configReverseSoftLimitEnable(true, Constants.CTRE_TIMEOUT);

    arm.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, ARM_PID_SLOT, Constants.CTRE_TIMEOUT);

    arm.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, Constants.CAN_STATUS_FREQ.HIGH);

    arm.configNominalOutputForward(NOMINAL_OUTPUT_FORWARD, Constants.CTRE_TIMEOUT);
    arm.configNominalOutputReverse(NOMINAL_OUTPUT_REVERSE, Constants.CTRE_TIMEOUT);
    arm.configPeakOutputForward(PEAK_OUTPUT_FORWARD, Constants.CTRE_TIMEOUT);
    arm.configPeakOutputReverse(PEAK_OUTPUT_REVERSE, Constants.CTRE_TIMEOUT);
    
    arm.configAllowableClosedloopError(ARM_PID_SLOT, 0, Constants.CTRE_TIMEOUT);

    arm.config_kP(ARM_PID_SLOT, ARM_NORMAL_P_VAL, Constants.CTRE_TIMEOUT);
    arm.config_kI(ARM_PID_SLOT, ARM_NORMAL_I_VAL, Constants.CTRE_TIMEOUT);
    arm.config_kD(ARM_PID_SLOT, ARM_NORMAL_D_VAL, Constants.CTRE_TIMEOUT);
    arm.config_kF(ARM_PID_SLOT, ARM_NORMAL_F_VAL, Constants.CTRE_TIMEOUT);
  }

  /**  sets the position of the entire arm */
  public void setPosition(ArmPosition armPos) {
    arm.set(ControlMode.Position, MercMath.degreesToEncoderTicks(armPos.degreePos));
  }

  public void moveArm(Supplier<Double> speedSupplier) {
    arm.set(ControlMode.PercentOutput, speedSupplier.get());
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("arm encoder", getArmPosition());
    SmartDashboard.putNumber("arm error", getError());

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

  
}
