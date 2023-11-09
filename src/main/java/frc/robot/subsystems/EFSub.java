// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class EFSub extends SubsystemBase {
  /** Creates a new EFSub. */
  //pneumatics
  static DoubleSolenoid piston;
  public EFSub() {
    //pneumatics
    piston = new DoubleSolenoid(Constants.pneumaticsConstants.pcm, Constants.pneumaticsConstants.PneumaticsForwardChannel, Constants.pneumaticsConstants.PneumaticsBackwardChannel);
  }
  public static void ToggleEF() {piston.toggle();}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
