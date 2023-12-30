package frc.robot.subsystems.elevator;

public interface ElevatorIO {       
    
    public double getEncoderPosition();
    public double getEncoderSpeed();
    public void setMotorSpeed(double speed);
    public void setEncoderPosition(double position);
    public void periodicUpdate();
    public double getElevatorCurrent();
}
