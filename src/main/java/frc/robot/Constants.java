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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class motorConstants {

    //rightMotors
    public static final int front_Right_MotorConstant = 1;
    public static final int back_Right_MotorConstant = 1;

    //leftMotors
    public static final int front_Left_MotorConstant = 1;
    public static final int back_Left_MotorConstant = 1;

    //armMotor
    public static final int armMotorId = 1;
    public static final double armP = 0;
    public static final double armI = 0;
    public static final double armD = 0;
    public static final double[] armPIDOutputRange = {-0.6, 0.6};

    //Gyro Constants
    public static final double gyro_kP = .005;
  


    

  }
  
  public static class miscConstants {
    //misc Bool


    //misc Int



    //idk continue the comments
  }
}
