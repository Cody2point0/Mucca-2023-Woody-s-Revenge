// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.motorConstants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
public class ArmSub extends SubsystemBase {

  CANSparkMax motor;
  SparkMaxAbsoluteEncoder encoder;
  SparkMaxPIDController pidController;
  enum Position {HOLDING,PLACING,STORING,PICKUP};







  /** Creates a new ArmSub. */
  public ArmSub() {

    motor = new CANSparkMax(Constants.motorConstants.armMotorId, MotorType.kBrushless);

    encoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    pidController = motor.getPIDController();
    pidController.setFeedbackDevice(encoder);
    pidController.setP(Constants.motorConstants.armP);
    pidController.setI(Constants.motorConstants.armI);
    pidController.setD(Constants.motorConstants.armD);
    pidController.setOutputRange(-0.6,0.6);
    pidController.setPositionPIDWrappingEnabled(false);



    //must stay at bottom
    motor.burnFlash()
    
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("EncoderPosition", encoder.getPosition());
      pidController.setReference(Rotation2d.fromDegrees(15),,0,aff);

    //this method will be called once per scheduler run
  }

  public void setArmPosition(Position pos) {

  }
  
  
}
