package team1403.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team1403.robot.subsystems.Arm;
import team1403.robot.subsystems.Wrist;
import team1403.robot.Constants;

public class ArmWristCommand extends Command {
    private Arm m_arm;
    private Wrist m_wrist;
    private BooleanSupplier m_up;
    private BooleanSupplier m_down; 

    public ArmWristCommand(Arm arm, Wrist wrist, BooleanSupplier up, BooleanSupplier down) {
        m_arm = arm;
        m_wrist = wrist;
        m_up = up;
        m_down = down;

        addRequirements(m_arm, m_wrist);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        System.out.println("askhgdkahsgda");
        if (m_up.getAsBoolean()) {
            m_wrist.setWristSetpoint(Constants.Wrist.kDriveSetpoint);
            if (m_wrist.isWristAtSetpoint()) {
                m_arm.setArmSetpoint(Constants.Arm.kDriveSetpoint);
            }  
        }
        if (m_down.getAsBoolean()) {
            m_wrist.setWristSetpoint(Constants.Wrist.kIntakeSetpoint);
            if (m_wrist.isWristAtSetpoint()) {
                m_arm.setArmSetpoint(Constants.Arm.kIntakeSetpoint);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
