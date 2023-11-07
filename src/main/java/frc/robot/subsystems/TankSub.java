// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class TankSub extends SubsystemBase {

  // private, only allows the TankSub subsystem to access these variables
  //TalonSRX, type of motor controller
  //frontRight, frontLeft, backRight, backLeft are all names of the variable
  //new TalonSRX(deviceNumer), creates the talon object and set's the decive number for access 
  private TalonSRX frontRight = new TalonSRX(Constants.motorConstants.front_Right_MotorConstant); 
  private TalonSRX frontLeft = new TalonSRX(Constants.motorConstants.front_Left_MotorConstant);
  private TalonSRX backRight = new TalonSRX(Constants.motorConstants.back_Right_MotorConstant);
  private TalonSRX backLeft = new TalonSRX(Constants.motorConstants.back_Left_MotorConstant);


  /** Creates a new driveTrain. */
  public TankSub() {

    // inverting motor outputs to prevent the robot from spinning in place
    frontLeft.setInverted(InvertType.InvertMotorOutput);
    backLeft.setInverted(InvertType.InvertMotorOutput);

    //backLeft TalonSRX follow the outputs given to frontLeft TalonSRX
    backLeft.follow(frontLeft);

    //backRight TalonSRX follows the outputs given to frontRight TalonSRX
    backRight.follow(frontRight);


  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  // creates a method named m_drive, intaking two double variables 
  public void drive( double drive, double turn){

    //setting voltage output to the motor controllers in terms of percent output ( 1 = 100%, .5 = 50%)
    frontRight.set(ControlMode.PercentOutput, (drive + turn) * .3);
    frontLeft.set(ControlMode.PercentOutput, (drive - turn) * .3);
    
  }


}
