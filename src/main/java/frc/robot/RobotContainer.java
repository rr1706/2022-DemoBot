// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.DriveByController;
import frc.robot.commands.FeedShooter;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.JoystickAnalogButton;
import frc.robot.utilities.JoystickAnalogButton.Side;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Shooter m_shooter = new Shooter();
  private final RunShooter m_runShooter = new RunShooter(m_shooter);

  private final Feeder m_feeder = new Feeder();
  private final FeedShooter m_feedShooter= new FeedShooter(m_feeder);

  private final XboxController m_controller = new XboxController(0);

  private final Drivetrain m_drive = new Drivetrain();
  private final DriveByController m_driveRobot = new DriveByController(m_drive, m_controller);

  private final Command m_autoCommand = new WaitCommand(20.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    m_drive.setDefaultCommand(m_driveRobot);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new POVButton(m_controller, 0)
        .whenPressed(() -> m_drive.resetOdometry(new Rotation2d(0.0)));

    new JoystickButton(m_controller, Button.kA.value).whenPressed(m_runShooter);
    new JoystickButton(m_controller, Button.kB.value).whenPressed(()->m_runShooter.cancel());

    new JoystickAnalogButton(m_controller, Side.kLeft).whileHeld(m_feedShooter);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
