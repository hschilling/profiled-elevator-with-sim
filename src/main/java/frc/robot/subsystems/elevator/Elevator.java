package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.elevator.ElevatorToHeightCommand;
import frc.robot.util.PIDUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;


public class Elevator extends ProfiledPIDSubsystem {

    private ElevatorIO elevatorIO;

    // PIDF tuned values:
    private static final double feedForward = 0.8;
    private static final double gravityCompensation = 0.025;

    // Trapezoidal profile constants and variables
    private static final double max_vel = 1.25; // m/s //0.2
    private static final double max_accel = 2.50; // m/s/s //0.4

    private static final Constraints constraints = new Constraints(max_vel, max_accel);

    private final ElevatorVisualizer elevatorVisualizer = new ElevatorVisualizer();

    public Elevator(ElevatorIO io) {
        super(new ProfiledPIDController(ElevatorConstants.kpPos, ElevatorConstants.kiPos, ElevatorConstants.kdPos,
                constraints));
        elevatorIO = io;
        elevatorIO.setEncoderPosition(0.);
    }

    @Override
    public void periodic() {
        super.periodic();
        elevatorIO.periodicUpdate();
        double currentPos = getMeasurement();
        double currentVel = getEncoderSpeed();
        SmartDashboard.putNumber("Elevator/goal (m)", getGoal());
        SmartDashboard.putNumber("Elevator/position (m)", currentPos);
        SmartDashboard.putNumber("Elevator/velocity (m per s)", currentVel);
        elevatorVisualizer.update(currentPos);
    }

    // returns height the elevator is at
    public double getMeasurement() {
        return elevatorIO.getEncoderPosition();
    }

    // returns speed of elevator
    public double getEncoderSpeed() {
        return elevatorIO.getEncoderSpeed();
    }

    public void setMotorSpeed(double speed) {
        SmartDashboard.putNumber("Elevator/motor speed (-1 to 1)", speed);
        elevatorIO.setMotorSpeed(speed);
    }

    public void setMotorSpeedGravityCompensation(double speed) {
        elevatorIO.setMotorSpeed(speed + gravityCompensation);
    }

    public double getElevatorCurrent() {
        return elevatorIO.getElevatorCurrent();
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        SmartDashboard.putNumber("Elevator/setpoint position (m)", setpoint.position);
        SmartDashboard.putNumber("Elevator/setpoint velocity (m per s)", setpoint.velocity);
        SmartDashboard.putNumber("Elevator/setpoint position error (m)", setpoint.position - getMeasurement());
        SmartDashboard.putNumber("Elevator/setpoint velocity error (m)", setpoint.velocity - getEncoderSpeed());

        // Calculate the feedforward from the setpoint
        double speed = feedForward * setpoint.velocity;
        // accounts for gravity in speed
        speed += gravityCompensation;
        elevatorIO.setMotorSpeed(speed);
    }

    // @Override
    // protected double getMeasurement() {
    //     return elevatorIO.getEncoderPosition();
    // }

    public double getGoal() {
        return m_controller.getGoal().position;
    }

    // Checks to see if elevators are within range of the setpoints
    public boolean atGoal() {
        return (PIDUtil.checkWithinRange(getGoal(), getMeasurement(), ElevatorConstants.HEIGHT_TOLERANCE));
    }

    public void setEncoderPosition(double position) {
        elevatorIO.setEncoderPosition(position);
    }

    public Command toHeightCommand(double heightMeters){
        final Command command = new ElevatorToHeightCommand(heightMeters, this);
        return command;
    }

    public Command toHeightInlineCommand(double heightMeters){
        final Command command = new PIDCommand(
            new PIDController(
                ElevatorConstants.kpPos,
                ElevatorConstants.kiPos,
                ElevatorConstants.kdPos),
            this::getMeasurement,
            // Setpoint
            heightMeters,
            // Pipe the output to the turning controls
            output -> this.setMotorSpeed(output),
            // Require the robot drive
            this);
        return command;
    }
}
