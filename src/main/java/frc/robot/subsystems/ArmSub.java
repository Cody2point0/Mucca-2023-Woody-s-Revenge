// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.SparkMaxAbsoluteEncoder;


public class ArmSub extends SubsystemBase {

  //motor
  CANSparkMax motor;
  SparkMaxAbsoluteEncoder encoder;


  /** Creates a new ArmSub. */
  public ArmSub() {



    //motor
    motor = new CANSparkMax(Constants.motorConstants.armMotorId, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.enableVoltageCompensation(12.0);
    motor.setSmartCurrentLimit(40);
    motor.setSecondaryCurrentLimit(60);
    motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    //encoder? we don't really need it haha..ha...ha.....
    encoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    //must stay at bottom
    motor.burnFlash();
  }

  @Override
  public void periodic() {
    //this method will be called once per scheduler run
  }


  public void setArmPower(double direct) {
    if (direct < Constants.OperatorConstants.kOpDeadband[0] || Constants.OperatorConstants.kOpDeadband[1] < direct)
      motor.set(direct * -0.17);
    else
      motor.set(0);

  }
}
