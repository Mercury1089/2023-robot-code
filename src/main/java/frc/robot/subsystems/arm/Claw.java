// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.util.MercMath;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  public static final int 
    CLAW_PID_SLOT = 0;

  public static final int
    REMOTE_DEVICE_0 = 0;

  public static final int
    ROLL_LOOP = 0;

  private static final double
    CLAW_NORMAL_P_VAL = 1.0,
    CLAW_NORMAL_I_VAL = 0.0,
    CLAW_NORMAL_D_VAL = 0.0,
    CLAW_NORMAL_F_VAL = 0.0;

  private final double 
    NOMINAL_OUTPUT_FORWARD = 0.02,
    NOMINAL_OUTPUT_REVERSE = -0.02,
    PEAK_OUTPUT_FORWARD = 0.2,
    PEAK_OUTPUT_REVERSE = -0.2;

  public final double
    CLAW_UPPER_LIMIT = 90,
    CLAW_LOWER_LIMIT = -90;
  // Need to find Gear ratio 30:1
  public final double GEAR_RATIO = 1;

  private TalonSRX claw;
  private PigeonIMU pigeon;

  public Claw() {
    claw = new TalonSRX(CAN.CLAW_TALON);
    // Configure Gyro
    pigeon = new PigeonIMU(CAN.ARM_GYRO);
    pigeon.configFactoryDefault();
    pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, Constants.CTRE_TIMEOUT);

    claw.configFactoryDefault();

    claw.setInverted(true);
    claw.setSensorPhase(true);

    //claw.configForwardSoftLimitThreshold(MercMath.degreesToEncoderTicks(CLAW_UPPER_LIMIT)*GEAR_RATIO, Constants.CTRE_TIMEOUT);
    //claw.configReverseSoftLimitThreshold(MercMath.degreesToEncoderTicks(CLAW_LOWER_LIMIT)*GEAR_RATIO, Constants.CTRE_TIMEOUT);
    //claw.configForwardSoftLimitEnable(true, Constants.CTRE_TIMEOUT);
    //claw.configReverseSoftLimitEnable(true, Constants.CTRE_TIMEOUT);

    claw.configRemoteFeedbackFilter(pigeon.getDeviceID(), RemoteSensorSource.Pigeon_Roll, REMOTE_DEVICE_0);
    claw.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, CLAW_PID_SLOT, Constants.CTRE_TIMEOUT);
    claw.configSelectedFeedbackCoefficient(Constants.UNITS.MAX_ROLL_DEGREES / Constants.UNITS.PIGEON_ROLL_UNITS, ROLL_LOOP, Constants.CTRE_TIMEOUT);

    claw.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, Constants.CAN_STATUS_FREQ.HIGH);

    claw.configNominalOutputForward(NOMINAL_OUTPUT_FORWARD, Constants.CTRE_TIMEOUT);
    claw.configNominalOutputReverse(NOMINAL_OUTPUT_REVERSE, Constants.CTRE_TIMEOUT);
    claw.configPeakOutputForward(PEAK_OUTPUT_FORWARD, Constants.CTRE_TIMEOUT);
    claw.configPeakOutputReverse(PEAK_OUTPUT_REVERSE, Constants.CTRE_TIMEOUT);
    claw.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0); 

    claw.configAllowableClosedloopError(CLAW_PID_SLOT, 0, Constants.CTRE_TIMEOUT);

    claw.config_kP(CLAW_PID_SLOT, CLAW_NORMAL_P_VAL, Constants.CTRE_TIMEOUT);
    claw.config_kI(CLAW_PID_SLOT, CLAW_NORMAL_I_VAL, Constants.CTRE_TIMEOUT);
    claw.config_kD(CLAW_PID_SLOT, CLAW_NORMAL_D_VAL, Constants.CTRE_TIMEOUT);
    claw.config_kF(CLAW_PID_SLOT, CLAW_NORMAL_F_VAL, Constants.CTRE_TIMEOUT);
    
  }



  public void calibratePigeon() {
    pigeon.enterCalibrationMode(PigeonIMU.CalibrationMode.BootTareGyroAccel);
  }

  public void levelClaw() {
    double roll = pigeon.getRoll();
    if (roll != 0) {
      claw.set(ControlMode.Position, 0 - roll);
    }
  }

  public void setClawPosition(ClawPosition clawPos) {
    claw.set(ControlMode.Position, MercMath.degreesToEncoderTicks(clawPos.degreePos));
  }

  public double getClawPosition() {
    return claw.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pigeon Roll", pigeon.getRoll());
    SmartDashboard.putNumber("Pigeon Roll(Talon)", claw.getSelectedSensorPosition(CLAW_PID_SLOT));
  }

  public enum ClawPosition {
    INSIDE(0),
    TOP_CONE(0),
    MID_CONE(0),
    TOP_CUBE(0),
    MID_CUBE(0),
    FLOOR(0),
    DOUBLE_SUBSTATION(0),
    FELL_OVER(0);

    public final double degreePos;

    ClawPosition(double degreePos) {
      this.degreePos = degreePos;
    }
  }
}
