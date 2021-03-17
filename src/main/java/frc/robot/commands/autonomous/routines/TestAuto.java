package frc.robot.commands.autonomous.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.TimedIntake;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class TestAuto extends SequentialCommandGroup {

    public TestAuto(DriveTrain driveTrain, Shooter shooter, Indexer indexer, Intake intake) {
        addCommands(
                new TimedIntake(intake, indexer, 3)
//                new SetIntakePiston(intake, true),
//                new WaitCommand(5),
//                new SetIntakePiston(intake, false)
        );
    }
}
