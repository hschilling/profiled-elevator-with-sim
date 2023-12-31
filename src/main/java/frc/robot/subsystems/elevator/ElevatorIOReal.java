package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.MotorUtil; 

public class ElevatorIOReal implements ElevatorIO {

    public static CANSparkMax elevatorMotorController;
    public static RelativeEncoder elevatorEncoder;

    public ElevatorIOReal()
    {
        elevatorMotorController = MotorUtil.createSparkMAX(ElevatorConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless, 
            Constants.NEO_CURRENT_LIMIT, false, true, 0.1);
        
        elevatorEncoder = elevatorMotorController.getEncoder();
        elevatorEncoder.setPositionConversionFactor(ElevatorConstants.METERS_PER_REVOLUTION);
        // dividng by 60 to convert meters per miniute to meters per seconds
        elevatorEncoder.setVelocityConversionFactor(ElevatorConstants.METERS_PER_REVOLUTION / 60);
    }

    @Override
    public double getEncoderPosition() {
        return elevatorEncoder.getPosition();
    }
    @Override
    public double getEncoderSpeed() {
        return elevatorEncoder.getVelocity();
    }
    
    @Override
    public void setMotorSpeed(double speed) {
        elevatorMotorController.set(speed);
    }

    @Override
    public void setEncoderPosition(double position) {
        elevatorEncoder.setPosition(position);
    }

    @Override
    public double getElevatorCurrent() {
        return elevatorMotorController.getOutputCurrent();
    }

    @Override
    public void periodicUpdate() {
        // Only code in here that relates a physical subsystem
        SmartDashboard.putNumber("elevator/Real motor temp (C)", elevatorMotorController.getMotorTemperature());
    }

}
