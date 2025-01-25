package team1403.robot.simulation;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import team1403.robot.subsystems.Elevator;
import team1403.robot.commands.ElevatorCommand;


public class MechanismSimulator {

    private final Mechanism2d panel;
    private final MechanismRoot2d root;
    private final MechanismLigament2d elevator;

    private final Elevator m_elevator;

    public MechanismSimulator (Elevator m_elevator) {

        this.m_elevator = m_elevator;
        this.panel = new Mechanism2d(Units.inchesToMeters(100), Units.inchesToMeters(100));
        this.root = panel.getRoot("elevator", Units.inchesToMeters(50), Units.inchesToMeters(3.75));
        this.elevator = root.append(new MechanismLigament2d("arm", 0, 0, 6, new Color8Bit(Color.kAliceBlue)));
    
    }

    public void periodic() {
        //Monday: add ElevatorState and add the simulation code again (it got deleted after the merge conflict)
    }

}
