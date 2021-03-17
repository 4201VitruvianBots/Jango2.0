package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.indexer.FeedAll;
import frc.robot.commands.turret.ShootOnTheMove;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;

public class SOTMWhileStill extends SequentialCommandGroup {
    public SOTMWhileStill(DriveTrain driveTrain, ShootOnTheMove shootOnTheMove, Indexer indexer) {
        addCommands(
                new SOTMForTime(shootOnTheMove, 10), // Aim & rev for 10 seconds
                new FeedAll(indexer) // Shoot
        );
    }
}
