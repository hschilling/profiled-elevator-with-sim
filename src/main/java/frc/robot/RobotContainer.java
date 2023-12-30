// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.elevator.ElevatorToHeightCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  public Elevator elevator;

  public RobotContainer() {
    setUpSubsystems();
    configureButtonBindings();
  }

  private void setUpSubsystems() {
    ElevatorIO elevatorIO;
    if (RobotBase.isSimulation()) {
      elevatorIO = new ElevatorIOSim();
    } else {
      elevatorIO = new ElevatorIOReal();
    }
    elevator = new Elevator(elevatorIO);
    elevator.disable();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    m_driverController
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  elevator.setGoal(0.2);
                  elevator.enable();
                },
                elevator));

    m_driverController
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  elevator.setGoal(0.5);
                  elevator.enable();
                },
                elevator));

    m_driverController
        .x()
        .whileTrue(
          elevator.toHeightInlineCommand(0.5));

    m_driverController
        .y()
        .whileTrue(
            elevator.toHeightInlineCommand(0.2));

    m_driverController
        .leftBumper()
        .whileTrue(
            Commands.run(
                () -> elevator.setMotorSpeed(-m_driverController.getRawAxis(1))));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PrintCommand("i am doing nothing");
  }
}
