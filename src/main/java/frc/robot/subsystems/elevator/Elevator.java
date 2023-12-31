package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.PIDUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class Elevator extends ProfiledPIDSubsystem {

    private ElevatorIO elevatorIO;

    private static final Constraints constraints = new Constraints(ElevatorConstants.max_vel,
            ElevatorConstants.max_accel);

    private final ElevatorVisualizer elevatorVisualizer = new ElevatorVisualizer();

    public Elevator(ElevatorIO io) {
        super(new ProfiledPIDController(ElevatorConstants.kpPos, ElevatorConstants.kiPos, ElevatorConstants.kdPos,
                constraints));


        getController()
                .setTolerance(ElevatorConstants.HEIGHT_TOLERANCE,ElevatorConstants.VELOCITY_TOLERANCE);
        
        elevatorIO = io;
        elevatorIO.setEncoderPosition(0.0);
    }

    @Override
    public void periodic() {
        // inside this next line, this is called
        //  useOutput(m_controller.calculate(getMeasurement()), m_controller.getSetpoint())
        // So what is the output of calculate?
        // SmartDashboard.putNumber("Elevator/Profiled PID setpoint pos", getController().calculate(getMeasurement(), getController().getSetpoint()));

        super.periodic();
        elevatorIO.periodicUpdate();
        double currentPos = getMeasurement();
        double currentVel = getEncoderSpeed();
        SmartDashboard.putNumber("Elevator/Profiled goal (m)", getGoal());
        SmartDashboard.putNumber("Elevator/position (m)", currentPos);
        SmartDashboard.putNumber("Elevator/velocity (m per s)", currentVel);
        SmartDashboard.putBoolean("Elevator/Profiled subsytem enabled", this.isEnabled());

        SmartDashboard.putNumber("Elevator/Profiled PID setpoint pos", getController().getSetpoint().position);
        SmartDashboard.putNumber("Elevator/Profiled PID position error", getController().getPositionError());
        SmartDashboard.putNumber("Elevator/Profiled PID velocity error", getController().getVelocityError());
        SmartDashboard.putNumber("Elevator/Profiled PID position tol", getController().getPositionTolerance());
        SmartDashboard.putNumber("Elevator/Profiled PID velocity tol", getController().getVelocityTolerance());
        SmartDashboard.putNumber("Elevator/Profiled PID goal pos", getController().getGoal().position);
        SmartDashboard.putBoolean("Elevator/Profiled PID position at Goal", getController().atGoal());
        SmartDashboard.putBoolean("Elevator/Profiled PID position at set point", getController().atSetpoint());
        
        elevatorVisualizer.update(currentPos);
    }

    // returns height the elevator is at. Required to override this
    @Override
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
        elevatorIO.setMotorSpeed(speed + ElevatorConstants.gravityCompensation);
    }

    public double getElevatorCurrent() {
        return elevatorIO.getElevatorCurrent();
    }

    // required to override this
    @Override
    protected void useOutput(double output, State setpoint) {
        SmartDashboard.putNumber("Elevator/Profiled useOutput output var (-1 to 1)", output);
        SmartDashboard.putNumber("Elevator/Profiled setpoint position (m)", setpoint.position);
        SmartDashboard.putNumber("Elevator/Profiled setpoint velocity (m per s)", setpoint.velocity);
        SmartDashboard.putNumber("Elevator/Profiled setpoint position error (m)", setpoint.position - getMeasurement());
        SmartDashboard.putNumber("Elevator/Profiled setpoint velocity error (m)",
                setpoint.velocity - getEncoderSpeed());


        // not doing anything with the position error ??


        // Calculate the feedforward from the setpoint
        double speed = ElevatorConstants.feedForward * setpoint.velocity;
        // accounts for gravity in speed
        speed += ElevatorConstants.gravityCompensation;


        speed += output;

        elevatorIO.setMotorSpeed(speed);
    }

    // @Override
    // protected double getMeasurement() {
    // return elevatorIO.getEncoderPosition();
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

    // factory method to make a PIDCommand for setting the elevator height
    public Command toHeightInlinePIDOnlyCommand(double heightMeters) {
        this.disable();
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
