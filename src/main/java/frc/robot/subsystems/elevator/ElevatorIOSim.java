package frc.robot.subsystems.elevator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.SimEncoder;

public class ElevatorIOSim implements ElevatorIO {
    public static SimEncoder elevatorSimEncoder;  // Our own class. Just a dumb class that gets and sets values representing an encoder
    public static ElevatorSim elevatorSim;  // from WPILib
    public double elevatorSpeed;
    public final static DCMotor elevatorGearbox = DCMotor.getNEO(1);
    // Simulated elevator constants and gearbox
    public static final double elevatorGearRatio = 9.0;
    public static final double elevatorDrumRadius = Units.inchesToMeters(1.0);
    public static final double elevatorCarriageMass = 5.5; // kg
    public static final double elevatorEncoderDistPerPulse = 2.0 * Math.PI * elevatorDrumRadius / 4096;

    public ElevatorIOSim() {
        elevatorSimEncoder = new SimEncoder("elevator");
        elevatorSim = new ElevatorSim(
                elevatorGearbox,
                elevatorGearRatio,
                elevatorCarriageMass,
                elevatorDrumRadius,
                Constants.ElevatorConstants.MIN_ELEVATOR_HEIGHT,
                Constants.ElevatorConstants.MAX_ELEVATOR_HEIGHT,
                true,
                VecBuilder.fill(Constants.ElevatorConstants.simMeasurementStdDev));

    }

    @Override
    public double getEncoderSpeed() {
        return elevatorSimEncoder.getSpeed();
    }

    @Override
    public void setMotorSpeed(double speed) {
        elevatorSpeed = speed;
    }

    @Override
    public double getEncoderPosition() {
        return elevatorSimEncoder.getDistance();
    }

    @Override
    public void setEncoderPosition(double position) {
        elevatorSimEncoder.setDistance(position);
    }

    @Override
    public double getElevatorCurrent() {
        return elevatorSim.getCurrentDrawAmps();
    }

    @Override
    public void periodicUpdate() {
        // sets input for elevator motor in simulation
        // SmartDashboard.putNumber("Elevator/periodicUpdate sim motor speed periodicUpdate (-1 to 1)", elevatorSpeed);
        elevatorSim.setInput(elevatorSpeed * RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("Elevator/periodicUpdate battery voltate", RobotController.getBatteryVoltage());
        // Next, we update it. The standard loop time is 20ms.
        elevatorSim.update(0.02);
        // Finally, we set our simulated encoder's readings
        SmartDashboard.putNumber("Elevator/periodicUpdate elevatorSim.getPositionMeters (m)", elevatorSim.getPositionMeters());

        elevatorSimEncoder.setDistance(elevatorSim.getPositionMeters());


        SmartDashboard.putNumber("Elevator/elevatorSimEncoder.getDistance (m)", elevatorSimEncoder.getDistance());

        // sets our simulated encoder speeds
        elevatorSimEncoder.setMotorSpeed(elevatorSim.getVelocityMetersPerSecond());

        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    }
}
