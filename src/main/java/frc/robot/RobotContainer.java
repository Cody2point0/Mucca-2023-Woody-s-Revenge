// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.EFSub;
import frc.robot.subsystems.TankSub;
import frc.robot.subsystems.ArmSub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // creates a new TankSub object
  private final TankSub m_DriveTrain = new TankSub();
  // creates a new xboxController object
  private final XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);
  CommandXboxController m_opController = new CommandXboxController(OperatorConstants.kOpControllerPort);

  ArmSub arm = new ArmSub();
  //creates RunCommand named driveCommand that takes the drive method from TankSub and sets the double parameters to the value of joystick controller
  RunCommand driveCommand = new RunCommand(()-> m_DriveTrain.drive(m_driverController.getLeftY(), m_driverController.getRightX()), m_DriveTrain);
  RunCommand ArmCommand = new RunCommand(()->arm.setArmPower(m_opController.getLeftY()),arm);
  // Replace with CommandPS4Controller or CommandJoystick if needed


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_opController.a().onTrue(new InstantCommand(EFSub::ToggleEF));

    arm.setDefaultCommand(ArmCommand);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
   //sets the defualt command to driveCommand, it will run constantly
     m_DriveTrain.setDefaultCommand(driveCommand);

     



     

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
   // null;
    // An example command will be run in autonomous
  //}
}
