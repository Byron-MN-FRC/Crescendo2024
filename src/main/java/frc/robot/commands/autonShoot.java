package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Spamp;

public class autonShoot extends Command {
    private final Spamp m_spamp;
    private final Timer m_time = new Timer();

    public autonShoot(Spamp subsystem) {
        m_spamp = subsystem;
        addRequirements(m_spamp);
    }

    @Override
    public void initialize() {
        // if (RobotContainer.getInstance().m_acquisition.isIntakeRetracted() &&
        // m_spamp.isNoteInSpamp()) {
        // m_spamp.deployShooter();
        /// }
        m_time.reset();
        m_time.start();
        m_spamp.deployShooter();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // if (!m_spamp.transferring){
        // if (m_spamp.isNoteInSpamp() || m_spamp.isNoteLeavingSpamp()) {
        //     m_spamp.deployShooter();
        //     m_spamp.speakerAutonShoot();
        // }
        
        if (m_time.get() > 1.1){
            m_spamp.shootSpeaker();
          } else {
            m_spamp.runTopShooter();
          }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_spamp.stopall();
        m_spamp.retractShooter();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return !m_spamp.isNoteLeavingSpamp() && !m_spamp.isNoteInSpamp()
        //         && !RobotContainer.getInstance().m_acquisition.isNoteInAcquisition();
        return m_spamp.isDoneShooting() && m_time.hasElapsed(1.5); 

    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}
