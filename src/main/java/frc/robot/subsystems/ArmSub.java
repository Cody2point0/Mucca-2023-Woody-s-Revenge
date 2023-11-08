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
  SparkMaxPIDController pidController;


  Compressor armCompressor;
  static DoubleSolenoid piston;
  static boolean solenoidIsExtended;








  /** Creates a new ArmSub. */
  public ArmSub() {




    //pneumatics
    armCompressor = new Compressor(Constants.pneumatics.pcm);
    piston = new DoubleSolenoid(Constants.pneumatics.pcm, 1, 2);

    //motor
    motor = new CANSparkMax(Constants.motorConstants.armMotorId, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.enableVoltageCompensation(12.0);
    motor.setSecondaryCurrentLimit(60);
    motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    //encoder? we don't really need it hahaha...ha.....
    encoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);


    //must stay at bottom
    motor.burnFlash();


    
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("EncoderPosition", encoder.getPosition());
      SmartDashboard.putBoolean("IsPistonExtended", solenoidIsExtended);
    //this method will be called once per scheduler run
  }

  public static void EFPICKUP () {
    piston.set(DoubleSolenoid.Value.kReverse);
    solenoidIsExtended = false;
  }

  public static void EFDROP () {
    piston.set(DoubleSolenoid.Value.kForward);
    solenoidIsExtended = true;
  }

  public void setArmPosition(double direct) {
    if (direct < Constants.OperatorConstants.kOpDeadband[0] || Constants.OperatorConstants.kOpDeadband[1] < direct) {
      motor.set(direct);
    }
  }
}
