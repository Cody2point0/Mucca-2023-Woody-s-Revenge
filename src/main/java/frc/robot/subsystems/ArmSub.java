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
public class ArmSub extends SubsystemBase {

  CANSparkMax motor;
  SparkMaxAbsoluteEncoder encoder;
  SparkMaxPIDController pidController;
  enum Position {HOLDING,PLACING,STORING,PICKUP};

  Compressor armCompressor;
  static DoubleSolenoid piston;








  /** Creates a new ArmSub. */
  public ArmSub() {




    motor = new CANSparkMax(Constants.motorConstants.armMotorId, MotorType.kBrushless);
    armCompressor = new Compressor(Constants.pneumatics.pcm);
    piston = new DoubleSolenoid(Constants.pneumatics.pcm, 1, 2);
    motor.restoreFactoryDefaults();

    encoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    pidController = motor.getPIDController();
    pidController.setFeedbackDevice(encoder);
    pidController.setP(Constants.motorConstants.armP);
    pidController.setI(Constants.motorConstants.armI);
    pidController.setD(Constants.motorConstants.armD);
    pidController.setOutputRange(-0.6,0.6);
    pidController.setPositionPIDWrappingEnabled(false);

    motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
    motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 100);
    motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 100);
    motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 100);
    motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, 100);
    motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, 100);
    motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6, 100);
    motor.enableVoltageCompensation(12.0);
    motor.setSecondaryCurrentLimit(60);


    //must stay at bottom
    motor.burnFlash();


    
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("EncoderPosition", encoder.getPosition());

    //this method will be called once per scheduler run
  }
  public void HOLDING(){

  }
  public void PLACING () {

  }
  public void STORING () {

  }
  public void PICKUP () {

  }
  public static void EFPICKUP () {
    piston.set(DoubleSolenoid.Value.kReverse);
  }

  public static void EFDROP () {
    piston.set(DoubleSolenoid.Value.kForward);
  }
  public int degToSensor(int deg) {
    return deg;
    //math
  }

  public int sensorToDeg(int sensor) {
    return sensor; //math
  }
  public void setArmPosition(Position pos) {
    switch(pos) {
      case PICKUP:

        //do something
      case STORING:
          //do something else

      case HOLDING:
        //do something crazy

      case PLACING:
        //do something bat-honky wilding bruv
    }


  }

  
  
}
