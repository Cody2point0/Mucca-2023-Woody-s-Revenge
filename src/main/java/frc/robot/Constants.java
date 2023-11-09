// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOpControllerPort = 1;
    public static final double[] kOpDeadband = {-0.1, 0.1};
  }

  public static class motorConstants {
    //commented values are reversed so the actual "back" of the robot is assumed as the front, per arm placement
    //rightMotors
    public static final int front_Right_MotorConstant = 18; //12
    public static final int back_Right_MotorConstant = 11; //13

    //leftMotors
    public static final int front_Left_MotorConstant = 13; //11
    public static final int back_Left_MotorConstant = 12; //18

    //armMotor
    public static final int armMotorId = 49;

    //Gyro Constants
    public static final double gyro_kP = .005;

  }

  public static class pneumaticsConstants {
    //REVPH is the long PCM that has big REV Robotics branding on it
    //CTREPCM is the other PCM - we use CTREPCM last checked
    public static final PneumaticsModuleType pcm = PneumaticsModuleType.CTREPCM;
    public static final int PneumaticsForwardChannel = 1;
    public static final int PneumaticsBackwardChannel = 2;
  }
}
