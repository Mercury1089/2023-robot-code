// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DS_USB;
import frc.robot.Constants.JOYSTICK_BUTTONS;
import frc.robot.auton.Autons;
import frc.robot.commands.drivetrain.SwerveOnJoysticks;
import frc.robot.subsystems.GamePieceLEDs;
import frc.robot.subsystems.GamePieceLEDs.LEDState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Claw;
import frc.robot.subsystems.arm.Intake;
import frc.robot.subsystems.arm.Wrist;
import frc.robot.subsystems.arm.Telescope;
import frc.robot.subsystems.arm.Wrist.WristPosition;
import frc.robot.subsystems.drivetrain.Drivetrain;

import java.util.function.Supplier;



import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController; 
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private CommandJoystick rightJoystick, leftJoystick;
  private CommandXboxController gamepad;

  private Trigger left1, left2, left3, left4, left5, left6, left7, left8, left9, left10, left11;
  private Trigger right1, right2, right3, right4, right5, right6, right7, right8, right9, right10, right11;
  private Trigger gamepadA, gamepadB, gamepadX, gamepadY, gamepadRB, gamepadLB, gamepadL3, gamepadBack, 
  gamepadStart, gamepadLeftStickButton, gamepadRightStickButton, gamepadLT, gamepadRT, gamepadPOVDown, gamepadPOVUpLeft, 
  gamepadPOVUp, gamepadPOVUpRight, gamepadPOVLeft, gamepadPOVRight, gamepadPOVDownRight, gamepadPOVDownLeft;

  private Supplier<Double> gamepadLeftX, gamepadLeftY, gamepadRightX, gamepadRightY, rightJoystickX, rightJoystickY, leftJoystickX, leftJoystickY;

  private Autons auton;
  private GamePieceLEDs LEDs;
  private Arm arm;
  private Telescope telescope;
  // private Claw claw;
  private Intake intake;
  private Wrist wrist;
  private Drivetrain drivetrain;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // buttons & bindings
    leftJoystick = new CommandJoystick(DS_USB.LEFT_STICK);
    rightJoystick = new CommandJoystick(DS_USB.RIGHT_STICK);
    gamepad = new CommandXboxController(DS_USB.GAMEPAD);
    configureBindings();
    
    // subsystems & sensors
    LEDs = new GamePieceLEDs();
    
    arm = new Arm();
    arm.setDefaultCommand(new RunCommand(() -> arm.setSpeed(gamepadRightY), arm));
    telescope = new Telescope();
    telescope.setDefaultCommand(new RunCommand(() -> telescope.setSpeed(gamepadRightX), telescope));
    // telescope.setDefaultCommand(
    //   new RunCommand(() -> telescope.setPosition(TelescopePosition.INSIDE), telescope)
    // );
    
    wrist = new Wrist();
    // wrist.setDefaultCommand(new RunCommand(() -> wrist.moveWrist(gamepadLeftY), wrist));
    wrist.setDefaultCommand(new RunCommand(() -> wrist.setPosition(WristPosition.INSIDE), wrist));
    // claw = new Claw();
    // claw.setDefaultCommand(new RunCommand(() -> claw.moveClaw(gamepadLeftX), claw));

    intake = new Intake(LEDs);

    drivetrain = new Drivetrain();
    drivetrain.setDefaultCommand(new SwerveOnJoysticks(drivetrain, leftJoystickX, leftJoystickY, rightJoystickX));
    drivetrain.resetGyro();

    // wrist.setDefaultCommand(new RunCommand(() -> wrist.moveWrist(gamepadLeftY), wrist));
    auton = new Autons(drivetrain, arm, telescope, wrist, intake, LEDs);

    gamepadA.onTrue(auton.getScorePieceMidCommand(arm, telescope, wrist));
    // gamepadB.onTrue(auton.getSubstationCommand(arm, telescope, wrist, claw));
    gamepadB.onTrue(auton.getShelfPickupCommand(arm, telescope, wrist));
    gamepadY.onTrue(auton.getScorePieceHighCommand(arm, telescope, wrist));
    gamepadX.onTrue(auton.getHybridBulldozeCommand(arm, telescope, wrist));
    gamepadLB.onTrue(auton.getBulldozeCommand(arm, telescope, wrist));
    gamepadRB.onTrue(auton.getTuckInCommand(arm, telescope, wrist));

    gamepadStart.and(gamepadBack).whileTrue(
      new ParallelCommandGroup(
        new RunCommand(() -> wrist.setPosition(WristPosition.INSIDE), wrist),
        new RunCommand(() -> telescope.setSpeed(() -> -1.6), telescope),
        new RunCommand(() -> arm.setSpeed(() -> -.8), arm)
      )
    );

    gamepadPOVLeft.or(gamepadPOVUpLeft).or(gamepadPOVDownLeft).onTrue(
      new RunCommand(() -> wrist.setSpeed(() -> 0.0), wrist)
    );
    gamepadLT.whileTrue(new RunCommand(() -> wrist.setSpeed(() -> -0.8), wrist));
    gamepadRT.whileTrue(new RunCommand(() -> wrist.setSpeed(() -> 0.8), wrist));

    // left1.onTrue(new RunCommand(() -> claw.close(LEDs), claw));
    left1.whileTrue(
      new RunCommand(() -> intake.setSpeed(Intake.IntakeSpeed.EJECT), intake).handleInterrupt(() -> intake.setSpeed(Intake.IntakeSpeed.STOP))
    );
    
    left3.onTrue(
      new RunCommand(() -> LEDs.lightUp(LEDState.PURPLE), LEDs)
    );
    //-38 wrist ramp pickup
    //25 claw ramp pickup

    
    left6.onTrue(new RunCommand(() -> LEDs.lightUp(LEDState.CELEBRATION), LEDs));
    left8.onTrue(new InstantCommand(() -> wrist.calibrate(), wrist).ignoringDisable(true));
    // left9.onTrue(new SwerveOnGyro(drivetrain, -1.75));
  
    // in honor of resetTurret
    left10.onTrue(new InstantCommand(() -> drivetrain.resetGyro(), drivetrain).ignoringDisable(true));
    left11.onTrue(new RunCommand(() -> drivetrain.lockSwerve(), drivetrain));

    // right1.onTrue(new RunCommand(() -> claw.open(), claw));
    right1.whileTrue(
      new RunCommand(() -> intake.setSpeed(Intake.IntakeSpeed.INTAKE), intake).handleInterrupt(() -> intake.setSpeed(Intake.IntakeSpeed.STOP))
    );
    
    right3.onTrue(new RunCommand(() -> LEDs.lightUp(LEDState.YELLOW), LEDs));
    
    right4.onTrue(new RunCommand(() -> wrist.setPosition(WristPosition.LEVEL), wrist));
    right5.onTrue(new RunCommand(() -> wrist.setPosition(WristPosition.LEVEL), wrist));
   
    right10.onTrue(new RunCommand(() -> wrist.setSpeed(() -> 0.0), wrist));
    right11.onTrue(new InstantCommand(() -> drivetrain.joyDrive(0.0, 0.0, 0.0), drivetrain));
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

        left1 = leftJoystick.button(JOYSTICK_BUTTONS.BTN1);
        left2 = leftJoystick.button(JOYSTICK_BUTTONS.BTN2);
        left3 = leftJoystick.button(JOYSTICK_BUTTONS.BTN3);
        left4 = leftJoystick.button(JOYSTICK_BUTTONS.BTN4);
        left5 = leftJoystick.button(JOYSTICK_BUTTONS.BTN5);
        left6 = leftJoystick.button(JOYSTICK_BUTTONS.BTN6);
        left7 = leftJoystick.button(JOYSTICK_BUTTONS.BTN7);
        left8 = leftJoystick.button(JOYSTICK_BUTTONS.BTN8);
        left9 = leftJoystick.button(JOYSTICK_BUTTONS.BTN9);
        left10 = leftJoystick.button(JOYSTICK_BUTTONS.BTN10);
        left11 = leftJoystick.button(JOYSTICK_BUTTONS.BTN11);

        right1 = rightJoystick.button(JOYSTICK_BUTTONS.BTN1);
        right2 = rightJoystick.button(JOYSTICK_BUTTONS.BTN2);
        right3 = rightJoystick.button(JOYSTICK_BUTTONS.BTN3);
        right4 = rightJoystick.button(JOYSTICK_BUTTONS.BTN4);
        right5 = rightJoystick.button(JOYSTICK_BUTTONS.BTN5);
        right6 = rightJoystick.button(JOYSTICK_BUTTONS.BTN6);
        right7 = rightJoystick.button(JOYSTICK_BUTTONS.BTN7);
        right8 = rightJoystick.button(JOYSTICK_BUTTONS.BTN8);
        right9 = rightJoystick.button(JOYSTICK_BUTTONS.BTN9);
        right10 = rightJoystick.button(JOYSTICK_BUTTONS.BTN10);
        right11 = rightJoystick.button(JOYSTICK_BUTTONS.BTN11);

        gamepadA = gamepad.a();
        gamepadB = gamepad.b();
        gamepadX = gamepad.x();
        gamepadY = gamepad.y();
        gamepadRB = gamepad.rightBumper();
        gamepadLB = gamepad.leftBumper();
        gamepadBack = gamepad.back();
        gamepadStart = gamepad.start();
        gamepadLeftStickButton = gamepad.leftStick();
        gamepadRightStickButton = gamepad.rightStick();
        gamepadLT = gamepad.leftTrigger();
        gamepadRT = gamepad.rightTrigger();
        
        gamepadPOVDown = gamepad.povDown();
        gamepadPOVUpLeft = gamepad.povUpLeft();
        gamepadPOVUp = gamepad.povUp();
        gamepadPOVUpRight = gamepad.povUpRight();
        gamepadPOVLeft = gamepad.povLeft();
        gamepadPOVRight = gamepad.povRight();
        gamepadPOVDownRight = gamepad.povDownRight();
        gamepadPOVDownLeft = gamepad.povDownLeft();

        gamepadLeftX = () -> gamepad.getLeftX();
        gamepadRightX = () -> gamepad.getRightX();
        gamepadLeftY = () -> -gamepad.getLeftY();
        gamepadRightY = () -> -gamepad.getRightY();

        leftJoystickX = () -> leftJoystick.getX();
        leftJoystickY = () -> leftJoystick.getY();
        rightJoystickX = () -> rightJoystick.getX();
        rightJoystickY = () -> rightJoystick.getY();


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Autons getAutonomous() {
    // An example command will be run in autonomous
    return auton;
  }
}
