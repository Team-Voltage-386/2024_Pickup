// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
  //Add in correct values
  public static final int kFrontPickupID = 14;
  public static final int kRearPickupID = 15;
  public static final int kLoaderMotorID = 0;
  public static final int kLoadedPieceDetectorChannel = 0;
  public static final int kSecondaryPieceDetectorChannel = 0;
  public static final int kPneumaticsModule = 0;
  //

  /** the indexes to address buttons on the controller */
  public static final class ControllerConstants {
    public static final double kDeadband = .1;
    // public static final Joystick kDriver = new Joystick(0);
    public static final Joystick kManipulator = new Joystick(1);

    public static final int kLeftVertical = 1;
    public static final int kRightVertical = 5;
    public static final int kLeftHorizontal = 0;
    public static final int kRightHorizontal = 4;
    public static final int kLeftTrigger = 2;
    public static final int kRightTrigger = 3;

    public static final int kA = 1;
    public static final int kB = 2;
    public static final int kX = 3;
    public static final int kY = 4;
    public static final int kLeftBumper = 5;
    public static final int kRightBumper = 6;
    public static final int kLeftOptions = 7;
    public static final int kRightOptions = 8;
    public static final int kLeftJoystickPressed = 9;
    public static final int kRightJoystickPressed = 10;
  }

  public static class OperatorConstants 
  {
  public static final Joystick kcont1 =  new Joystick(0);
  public static final int kLEDPort = 4;

    public static final int kLimitSwitchPort = 0;
    public static final int kPushButton = 2;
    public static final int kOGSparkPort = 3;
    public static final int CANTalonSRX = 2;
    public static final int SparkMax = 1;
    public static final int PCM = 0;
    public static final int UltraPingDIOPort = 8;  // ultrasonic Ping port
    public static final int UltraEchoDIOPort = 9;  // ultrasonic Echo port
    public static final int Servo = 0;
    public static final int kIMUid = 12;  // pigeon
    public static final int SolenoidportForward = 3;
    public static final int SolenoidportBackward = 4;
    public static final int kDriverControllerPort = 0;
    public static final int kManipulatorControllerPort = 0;

    public static final int kLeftVertical = 1;
    public static final int kRightVertical = 5;
    public static final int kLeftHorizontal = 0;
    public static final int kRightHorizontal = 4;
    public static final int kLeftTrigger = 2;
    public static final int kRightTrigger = 3;

    public static final int kA = 1;
    public static final int kB = 2;
    public static final int kX = 3;
    public static final int kY = 4;
    public static final int kLeftBumper = 5;
    public static final int kRightBumper = 6;
    public static final int kLeftOptions = 7;
    public static final int kRightOptions = 8;
    public static final int kLeftJoystickPressed = 9;
    public static final int kRightJoystickPressed = 10;
    



  }
}
