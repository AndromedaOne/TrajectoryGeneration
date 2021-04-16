package frc.robot.BetterTrajectoryGenerators;

import java.util.List;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;

public abstract class BetterStatesGeneratorBase {
    protected List<State> m_states;
    
    protected BetterStatesGeneratorBase(List<State> states) {
        m_states = states;
    }

    public abstract List<State> getBetterStates();

    
}
