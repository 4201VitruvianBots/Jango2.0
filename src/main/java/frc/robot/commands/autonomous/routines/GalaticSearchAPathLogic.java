package frc.robot.commands.autonomous.routines;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GalaticSearchAPathLogic {
    public GalaticSearchAPathLogic(Vision vision) {
        if (Math.abs(vision.getPowerCellX()) < 20) {
            SmartDashboard.putString("vision_debug", "Powercell Found");
        } else {
            SmartDashboard.putString("vision_debug", "404: Powercell Not Found");
        }
    }
}
