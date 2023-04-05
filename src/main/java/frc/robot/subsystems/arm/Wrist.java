// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class Wrist extends SubsystemBase {
  /** Creates a new wrist. */

  public final double THRESHOLD_DEGREES = 3.0;
  public static final int 
    WRIST_PID_SLOT = 0;

  public static final int
    REMOTE_DEVICE_0 = 0;

  public static final int
    ROLL_LOOP = 0;

  private static final double
    WRIST_NORMAL_P_VAL = 1.0 / 10.0 * 1024.0,
    WRIST_NORMAL_I_VAL = 0.0,
    WRIST_NORMAL_D_VAL = 0.0,
    WRIST_NORMAL_F_VAL = 0.0;

  private final double 
    NOMINAL_OUTPUT_FORWARD = 0.02,
    NOMINAL_OUTPUT_REVERSE = -0.02,
    PEAK_OUTPUT_FORWARD = 1.0,
    PEAK_OUTPUT_REVERSE = -1.0;

  public final double
    WRIST_UPPER_LIMIT = 90,
    WRIST_LOWER_LIMIT = -90;
  // Need to find Gear ratio 30:1

  private VictorSPX wrist;
  private PigeonIMU pigeon;

  public Wrist() {
    wrist = new VictorSPX(CAN.WRIST_TALON);
    // Configure Gyro
    pigeon = new PigeonIMU(CAN.ARM_GYRO);
    pigeon.configFactoryDefault();
    pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, Constants.CTRE.TIMEOUT_MS);

    wrist.configFactoryDefault();

    wrist.setSensorPhase(true);
    wrist.setInverted(false);
    wrist.setNeutralMode(NeutralMode.Brake);
    wrist.configRemoteFeedbackFilter(pigeon.getDeviceID(), RemoteSensorSource.Pigeon_Roll, REMOTE_DEVICE_0);
    wrist.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, WRIST_PID_SLOT, Constants.CTRE.TIMEOUT_MS);
    wrist.configSelectedFeedbackCoefficient(Constants.UNITS.MAX_ROLL_DEGREES / Constants.UNITS.PIGEON_ROLL_UNITS, WRIST_PID_SLOT, Constants.CTRE.TIMEOUT_MS);

    wrist.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, Constants.CAN_STATUS_FREQ.HIGH);

    wrist.configNominalOutputForward(NOMINAL_OUTPUT_FORWARD, Constants.CTRE.TIMEOUT_MS);
    wrist.configNominalOutputReverse(NOMINAL_OUTPUT_REVERSE, Constants.CTRE.TIMEOUT_MS);
    wrist.configPeakOutputForward(PEAK_OUTPUT_FORWARD, Constants.CTRE.TIMEOUT_MS);
    wrist.configPeakOutputReverse(PEAK_OUTPUT_REVERSE, Constants.CTRE.TIMEOUT_MS);
    //wrist.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0); 

    wrist.configAllowableClosedloopError(WRIST_PID_SLOT, 0, Constants.CTRE.TIMEOUT_MS);

    wrist.config_kP(WRIST_PID_SLOT, WRIST_NORMAL_P_VAL, Constants.CTRE.TIMEOUT_MS);
    wrist.config_kI(WRIST_PID_SLOT, WRIST_NORMAL_I_VAL, Constants.CTRE.TIMEOUT_MS);
    wrist.config_kD(WRIST_PID_SLOT, WRIST_NORMAL_D_VAL, Constants.CTRE.TIMEOUT_MS);
    wrist.config_kF(WRIST_PID_SLOT, WRIST_NORMAL_F_VAL, Constants.CTRE.TIMEOUT_MS);
    
    wrist.selectProfileSlot(WRIST_PID_SLOT, Constants.CTRE.PRIMARY_PID_LOOP);
  }

  public void setSpeed(Supplier<Double> speedSupplier) {
    wrist.set(ControlMode.PercentOutput, speedSupplier.get());
  }

  public void calibratePigeon() {
    pigeon.enterCalibrationMode(PigeonIMU.CalibrationMode.BootTareGyroAccel);
  }

  public void setPosition(WristPosition wristPos) {
    wrist.set(ControlMode.Position, wristPos.degreePos);
  }

  public void incrementWrist(double increment) {
    wrist.set(ControlMode.Position, wrist.getSelectedSensorPosition(WRIST_PID_SLOT) + increment);
  }

  public double getWristPosition() {
    return wrist.getSelectedSensorPosition();
  }

  public boolean isAtPosition(WristPosition pos) {
    return Math.abs(getWristPosition() - pos.degreePos) < THRESHOLD_DEGREES;
  }

  public void calibrate() {
    pigeon.enterCalibrationMode(CalibrationMode.BootTareGyroAccel);
  }

  public boolean isReady() {
    return (pigeon.getState() == PigeonState.Ready);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Roll", getWristPosition());
    SmartDashboard.putBoolean("Wrist Ready", isReady());
  }

  public enum WristPosition {
    INSIDE(64.0),
    FLOOR(-25.0),
    LEVEL(0.0),
    BULLDOZER(10.0),
    HIGH_SCORE(20.0),
    MID_SCORE(20.0),
    RAMP(38),
    FELL_OVER(0);

    public final double degreePos;

    WristPosition(double degreePos) {
      this.degreePos = degreePos;
    }
  }
}
