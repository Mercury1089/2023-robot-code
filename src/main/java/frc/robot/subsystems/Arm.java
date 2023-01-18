// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MercMath;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  TalonSRX arm;
  public Arm() {
    arm = new TalonSRX(0);
    
    // these lines look kinda important 
   // arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PID.PRIMARY_PID_LOOP, RobotMap.CTRE_TIMEOUT);
   // arm.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, RobotMap.CAN_STATUS_FREQ.HIGH);
   // arm.configAllowableClosedloopError(RobotMap.PID.PRIMARY_PID_LOOP, 0, RobotMap.CTRE_TIMEOUT);
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
}
