// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Claw;
import frc.robot.subsystems.arm.Telescope;

public class ManualArm extends CommandBase {

  private Arm arm;
  private Telescope telescope;
  private Claw claw;
  private Supplier<Double> gamepadRightY;
  /** Creates a new ManualArm. */
  public ManualArm(Arm arm, Telescope telescope, Claw claw, Supplier<Double> gamepadRightY) {
    this.arm = arm;
    this.telescope = telescope;
    this.claw = claw;
    this.gamepadRightY = gamepadRightY;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // this.claw.levelClaw();
    // double degrees = 90 * gamepadRightY.get();
    // arm.setPosition(degrees);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
