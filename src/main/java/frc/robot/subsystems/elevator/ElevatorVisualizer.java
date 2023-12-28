package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

public class ElevatorVisualizer {
    private final Mechanism2d mech;
    private final MechanismRoot2d root;
    private final MechanismLigament2d elevatorMechanism;

    public ElevatorVisualizer()
    {
        mech = new Mechanism2d(1, 1);
        root = mech.getRoot("root", 0.3, 0);
        elevatorMechanism = root.append(new MechanismLigament2d("elevator", Constants.ElevatorConstants.MIN_ELEVATOR_HEIGHT, 50));
        elevatorMechanism.setColor(new Color8Bit(0, 204, 255));
        elevatorMechanism.setLineWeight(20);
    }

    public void update(double currentPos){
        elevatorMechanism.setLength(Constants.ElevatorConstants.MIN_ELEVATOR_HEIGHT + currentPos);
    }
}
