// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.util.MercMath;

public class Wrist extends SubsystemBase {
  /** Creates a new wrist. */
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

  private TalonFX wrist;
  private PigeonIMU pigeon;

  public Wrist() {
    wrist = new TalonFX(CAN.WRIST_TALON);
    // Configure Gyro
    pigeon = new PigeonIMU(CAN.ARM_GYRO);
    pigeon.configFactoryDefault();
    pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, Constants.CTRE_TIMEOUT);

    wrist.configFactoryDefault();

    wrist.setSensorPhase(true);
    wrist.setInverted(false);

    wrist.configRemoteFeedbackFilter(pigeon.getDeviceID(), RemoteSensorSource.Pigeon_Roll, REMOTE_DEVICE_0);
    wrist.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, WRIST_PID_SLOT, Constants.CTRE_TIMEOUT);
    wrist.configSelectedFeedbackCoefficient(Constants.UNITS.MAX_ROLL_DEGREES / Constants.UNITS.PIGEON_ROLL_UNITS, WRIST_PID_SLOT, Constants.CTRE_TIMEOUT);

    wrist.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, Constants.CAN_STATUS_FREQ.HIGH);

    wrist.configNominalOutputForward(NOMINAL_OUTPUT_FORWARD, Constants.CTRE_TIMEOUT);
    wrist.configNominalOutputReverse(NOMINAL_OUTPUT_REVERSE, Constants.CTRE_TIMEOUT);
    wrist.configPeakOutputForward(PEAK_OUTPUT_FORWARD, Constants.CTRE_TIMEOUT);
    wrist.configPeakOutputReverse(PEAK_OUTPUT_REVERSE, Constants.CTRE_TIMEOUT);
    //wrist.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0); 

    wrist.configAllowableClosedloopError(WRIST_PID_SLOT, 0, Constants.CTRE_TIMEOUT);

    wrist.config_kP(WRIST_PID_SLOT, WRIST_NORMAL_P_VAL, Constants.CTRE_TIMEOUT);
    wrist.config_kI(WRIST_PID_SLOT, WRIST_NORMAL_I_VAL, Constants.CTRE_TIMEOUT);
    wrist.config_kD(WRIST_PID_SLOT, WRIST_NORMAL_D_VAL, Constants.CTRE_TIMEOUT);
    wrist.config_kF(WRIST_PID_SLOT, WRIST_NORMAL_F_VAL, Constants.CTRE_TIMEOUT);
    
  }

  public void moveWrist(Supplier<Double> speedSupplier) {
    wrist.set(ControlMode.PercentOutput, speedSupplier.get());
  }

  public void calibratePigeon() {
    pigeon.enterCalibrationMode(PigeonIMU.CalibrationMode.BootTareGyroAccel);
  }

  public void setWristPosition(WristPosition wristPos) {
    wrist.set(ControlMode.Position, wristPos.degreePos);
  }

  public double getWristPosition() {
    return wrist.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Roll", pigeon.getRoll());
    SmartDashboard.putNumber("Wrist Roll(Talon)", wrist.getSelectedSensorPosition(WRIST_PID_SLOT));
  }

  public enum WristPosition {
    INSIDE(58.0),
    FLOOR(-25.0),
    LEVEL(0.0),
    FELL_OVER(0);

    public final double degreePos;

    WristPosition(double degreePos) {
      this.degreePos = degreePos;
    }
  }
}
