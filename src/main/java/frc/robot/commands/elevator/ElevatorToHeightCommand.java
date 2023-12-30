package frc.robot.commands.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator;

/** A command that will turn the robot to the specified angle. */
public class ElevatorToHeightCommand extends PIDCommand {
  /**
   * Turns to robot to the specified angle.
   *
   * @param heightMeters The height to go to
   * @param elevator              The elevator subsystem to use
   */

   private Elevator elevator ;
  public ElevatorToHeightCommand(double heightMeters, Elevator elevator) {
    super(
        new PIDController(ElevatorConstants.kpPos, ElevatorConstants.kiPos, ElevatorConstants.kdPos),
        // Close loop on heading
        elevator::getMeasurement,
        // Set reference to target
        heightMeters,
        // Pipe output to turn robot
        output -> elevator.setMotorSpeed(output + ElevatorConstants.gravityCompensation),
        // Require the drive
        elevator);
    
    this.elevator = elevator;
    System.out.println("ElevatorToHeightCommand constructor");

    SmartDashboard.putData("Elevator/PID Controller", getController());

    // Set the controller tolerance - the delta tolerance ensures the robot is
    // stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(ElevatorConstants.HEIGHT_TOLERANCE);
  }

  @Override
  public void execute(){
    System.out.println("execute");
    SmartDashboard.putNumber("Elevator/PID setPoint (m)", getController().getSetpoint());
    SmartDashboard.putNumber("Elevator/measurement (m)", elevator.getMeasurement());
    // getController().getSetpoint();
    super.execute();
  }
  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    System.out.println("is finished checking");
    SmartDashboard.putBoolean("Elevator/PID at setPoint", getController().atSetpoint());
    return false;
    // if (getController().atSetpoint())
    //   elevator.setMotorSpeed(0.0);
    // return getController().atSetpoint();
  }
}