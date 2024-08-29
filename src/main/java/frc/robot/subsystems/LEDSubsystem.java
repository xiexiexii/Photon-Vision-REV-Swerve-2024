package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;;

public class LEDSubsystem extends SubsystemBase {
    Spark blinkin;
    public LEDSubsystem(){
        blinkin = new Spark(LEDConstants.blinkinPort);
    }

    public void periodic(){
        setConfetti();
    }

    public void setRainbowParty() {
        blinkin.set(-0.97);
    }

    public void setConfetti() {
        blinkin.set(-0.87);
    }

    public void setBlueShot() {
        blinkin.set(-0.83);
    }

    public void setRainbowTwinkle() {
        blinkin.set(-0.55);
    }

    public void turnOff() {
        blinkin.set(0);
    }

    public void setHotPink() {
        blinkin.set(0.57);
    }

    public void setAqua() {
        blinkin.set(0.81);
    }
}