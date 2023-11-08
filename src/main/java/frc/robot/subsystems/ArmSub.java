// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.motorConstants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.RobotContainer;

public class ArmSub extends SubsystemBase {

  CANSparkMax motor;
  SparkMaxAbsoluteEncoder encoder;


  Compressor armCompressor;
  static DoubleSolenoid piston;
  static boolean wantSolenoidOut;







  /** Creates a new ArmSub. */
  public ArmSub() {





    //pneumatics
    piston = new DoubleSolenoid(Constants.pneumatics.pcm, 1, 2);
    wantSolenoidOut = false;

    //motor
    motor = new CANSparkMax(Constants.motorConstants.armMotorId, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.enableVoltageCompensation(12.0);
    motor.setSecondaryCurrentLimit(60);
    motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    //encoder? we don't really need it haha..ha...ha.....
    encoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);



    //must stay at bottom
    motor.burnFlash();
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("EncoderPosition", encoder.getPosition());
      SmartDashboard.putBoolean("IsPistonTryingExtend", wantSolenoidOut);

      //updated controls to test for Toggle-style single button EF control
      if (wantSolenoidOut) {EFOUT();}
      else {EFIN();}

    //this method will be called once per scheduler run
  }
  public static void ToggleEF() {
    wantSolenoidOut = !wantSolenoidOut;
  }
  public static void EFIN() {
    piston.set(DoubleSolenoid.Value.kReverse);
  }

  public static void EFOUT() {
    piston.set(DoubleSolenoid.Value.kForward);
  }

  public void setArmPosition(double direct) {
    if (direct < Constants.OperatorConstants.kOpDeadband[0] || Constants.OperatorConstants.kOpDeadband[1] < direct) {
      motor.set(direct);
    }
  }
}
