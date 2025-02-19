package team1403.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoHelper {
    
    public static Command getMoveAuto() {
        return AutoBuilder.buildAuto("MoveAuto");
    }
}
