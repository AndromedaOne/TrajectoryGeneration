package frc.robot.BetterTrajectoryGenerators;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.differentiation.DerivativeStructure;
import org.apache.commons.math3.analysis.differentiation.FiniteDifferencesDifferentiator;
import org.apache.commons.math3.analysis.differentiation.UnivariateDifferentiableFunction;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import frc.robot.Logging.Logging;

public class CalculatedCurvatures extends BetterStatesGeneratorBase {

    FiniteDifferencesDifferentiator m_differentiator;
    public CalculatedCurvatures(List<State> states) {
        super(states);
        
    }

    @Override
    public List<State> getBetterStates() {
        Trajectory trajectory = new Trajectory(m_states);
        m_differentiator = new FiniteDifferencesDifferentiator(3, 0.1, 0, trajectory.getTotalTimeSeconds());
        UnivariateFunction xValues = new UnivariateFunction() {
            public double value(double t) {
                return trajectory.sample(t).poseMeters.getX();
            }
        };
        UnivariateFunction yValues = new UnivariateFunction() {
            public double value(double t) {
                return trajectory.sample(t).poseMeters.getY();
            }
        };

        UnivariateDifferentiableFunction completeX = m_differentiator.differentiate(xValues);
        UnivariateDifferentiableFunction completeY = m_differentiator.differentiate(yValues); 
        List<State> betterStates = new ArrayList<State>();
        for(State state : m_states) {
            DerivativeStructure derivativeStructure = new DerivativeStructure(1, 2, 0, state.timeSeconds);
            DerivativeStructure x = completeX.value(derivativeStructure);
            double dx = x.getPartialDerivative(1);
            double dxSquared = x.getPartialDerivative(2);
            DerivativeStructure y = completeY.value(derivativeStructure);
            double dy = y.getPartialDerivative(1);
            double dySquared = y.getPartialDerivative(2);

            double newCurvature = getCurvature(dx, 
                dxSquared, 
                dy, 
                dySquared);
            
            //newCurvature *= Math.signum(state.curvatureRadPerMeter);
            State betterState = new State(
                state.timeSeconds,
                state.velocityMetersPerSecond,
                state.accelerationMetersPerSecondSq,
                state.poseMeters,
                newCurvature
            );
            betterStates.add(betterState);
        }

        return betterStates;
    }

    public static double getCurvature(double dx, double dy, double dxSquared, double dySquared) {
        return(Math.abs(dx * dySquared - dy * dxSquared) / Math.pow(Math.pow(dx, 2) + Math.pow(dy, 2), 1.5));
    }


    
}
