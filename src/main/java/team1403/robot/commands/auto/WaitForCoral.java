package team1403.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.Blackbox;

public class WaitForCoral extends Command {

    @Override
    public boolean isFinished() {
        return Blackbox.isCoralLoaded();
    }
    
}
