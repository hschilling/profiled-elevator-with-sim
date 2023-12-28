package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;
import frc.robot.util.SimEncoder;

public class ElevatorIOSim implements ElevatorIO {
    public static SimEncoder elevatorSimEncoder;
    public static ElevatorSim elevatorSim;
    public double elevatorSpeed;
    public static CANSparkMax elevatorMotorControllerRight;
    public final static DCMotor elevatorGearbox = DCMotor.getNEO(2);
    // Simulated elevator constants and gearbox
    public static final double elevatorGearRatio = 9.0;
    public static final double elevatorDrumRadius = Units.inchesToMeters(1.0);
    public static final double elevatorCarriageMass = 5.5; // kg
    // The simulated encoder will return
    public static final double elevatorEncoderDistPerPulse = 2.0 * Math.PI * elevatorDrumRadius / 4096;

    public ElevatorIOSim()
    {
        elevatorSimEncoder = new SimEncoder("elevator");
        elevatorSim = new ElevatorSim(
        elevatorGearbox,
        elevatorGearRatio,
        elevatorCarriageMass,
        elevatorDrumRadius,
        Constants.ElevatorConstants.MIN_ELEVATOR_HEIGHT,
        Constants.ElevatorConstants.MAX_ELEVATOR_HEIGHT,
        true,
        VecBuilder.fill(0.001)
      );
    
    }
    @Override
    public double getEncoderSpeed() {
        return elevatorSimEncoder.getSpeed();
    }

    @Override
    public void setSpeed(double speed) {
        elevatorSpeed = speed;
    }

    @Override
    public double getEncoderPosition() {
        return elevatorSimEncoder.getDistance();
    }

    @Override
    public void setPosition(double position) {
        elevatorSimEncoder.setDistance(position);
    }

    @Override
    public double getElevatorCurrent() {
        return elevatorSim.getCurrentDrawAmps(); // TODO What is is?
    }

    @Override
    public void periodicUpdate() {
        // sets input for elevator motor in simulation
        elevatorSim.setInput(elevatorSpeed * RobotController.getBatteryVoltage());
        // Next, we update it. The standard loop time is 20ms.
        elevatorSim.update(0.02);
        // Finally, we set our simulated encoder's readings
        elevatorSimEncoder.setDistance(elevatorSim.getPositionMeters());
        // sets our simulated encoder speeds
        elevatorSimEncoder.setSpeed(elevatorSim.getVelocityMetersPerSecond());

        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    } 
}



    
