package frc.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.subsystems.Drivetrain;;

public class Auton extends SequentialCommandGroup{
    Drivetrain dt;
    public Auton(){
        SmartDashboard.putString("Auton mode", "Defualt");
        this.addCommands(new DriveStraight());
        this.addCommands(new DriveStraight());

    }
}
