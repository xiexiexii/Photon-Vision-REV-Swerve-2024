package frc.robot.commands;

import java.util.Timer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class LEDColorChangeCommand extends Command {
    
    // Instantiate Stuff
    LEDSubsystem m_LEDSubsystem;
    Timer timer = new Timer();

    public LEDColorChangeCommand(LEDSubsystem ledSubsystem) {
        
        // Definitions and setting parameters are equal to members!
        m_LEDSubsystem = ledSubsystem;
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        m_LEDSubsystem.setBlueShot();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
