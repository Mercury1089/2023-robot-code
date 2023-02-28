// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GamePieceLEDs extends SubsystemBase {

  private Spark blinkin;
  private LEDState gamePieceState;
  private GenericEntry booleanBox = null; 
  /** Creates a new GamePieceLEDs. */
  public GamePieceLEDs() {
    this.blinkin = new Spark(0);
    gamePieceState = LEDState.OFF;

  }

  /** set by the buttons
   * - saves the game state
   * - sets the SD boolean box to color
   * - sets the physical LED
  */
  public void lightUp(LEDState ledState) {
   this.gamePieceState = ledState;

   if (this.gamePieceState.gamePiece == GamePiece.CONE) {
    this.booleanBox.setBoolean(false);
   } else if (this.gamePieceState.gamePiece == GamePiece.CUBE) {
    this.booleanBox.setBoolean(true);
   }

   blinkin.set(gamePieceState.colorValue);
  }

  public GamePiece getGameState() {
    return this.gamePieceState.gamePiece;
  }

  public void setBooleanBox(GenericEntry booleanBox) {
    this.booleanBox = booleanBox;
  }

  public enum LEDState {
    OFF(0.99, GamePiece.NONE), YELLOW(0.69, GamePiece.CONE), PURPLE(0.91, GamePiece.CUBE), CELEBRATION(-0.87, GamePiece.NONE);

    public final double colorValue;
    public final GamePiece gamePiece;

    LEDState(double colorValue, GamePiece gamePiece)  {
        this.colorValue = colorValue;
        this.gamePiece = gamePiece;
    }
  }

  public enum GamePiece {
    NONE,
    CUBE,
    CONE
  }

  @Override
  public void periodic() {
  }
}
