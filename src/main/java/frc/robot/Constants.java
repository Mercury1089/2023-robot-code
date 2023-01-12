// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class DS_USB {
    public static final int LEFT_STICK = 0, RIGHT_STICK = 1, GAMEPAD = 2;

    private DS_USB() {
    }
  }

  public static class GAMEPAD_BUTTONS {
    public static final int A = 1;
    public static final int B = 2;
    public static final int X = 3;
    public static final int Y = 4;
    public static final int LB = 5;
    public static final int RB = 6;
    public static final int BACK = 7;
    public static final int START = 8;
    public static final int L3 = 9;
    public static final int R3 = 10;

    private GAMEPAD_BUTTONS() {
    }
  }

  /**
  *From 0 in the up direction, and counter clockwise in degrees (right is 90 and upper left is 315).
  */
  public static class GAMEPAD_POV {
      public static final int MIDDLE = -1;
      public static final int UP = 0;
      public static final int UP_RIGHT = 45;
      public static final int RIGHT = 90;
      public static final int DOWN_RIGHT = 135;
      public static final int DOWN = 180;
      public static final int DOWN_LEFT = 225;
      public static final int LEFT = 270;
      public static final int UP_LEFT = 315;
  }
  public static class JOYSTICK_BUTTONS {
    public static final int BTN1 = 1;
    public static final int BTN2 = 2;
    public static final int BTN3 = 3;
    public static final int BTN4 = 4;
    public static final int BTN5 = 5;
    public static final int BTN6 = 6;
    public static final int BTN7 = 7;
    public static final int BTN8 = 8;
    public static final int BTN9 = 9;
    public static final int BTN10 = 10;
    public static final int BTN11 = 11;

    public JOYSTICK_BUTTONS() {
    }
  }

  public static class GAMEPAD_AXIS {
    public static final int leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX = 4, rightY = 5;

    private GAMEPAD_AXIS() {
    }
  }

}
