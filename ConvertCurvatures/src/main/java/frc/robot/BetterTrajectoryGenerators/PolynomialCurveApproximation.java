package frc.robot.BetterTrajectoryGenerators;

import java.util.List;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.differentiation.DerivativeStructure;
import org.apache.commons.math3.analysis.differentiation.FiniteDifferencesDifferentiator;
import org.apache.commons.math3.analysis.differentiation.UnivariateDifferentiableFunction;
import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoints;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import frc.robot.Logging.Logging;

public abstract class PolynomialCurveApproximation extends BetterStatesGeneratorBase {

    protected PolynomialFunction m_xApprox;
    protected PolynomialFunction m_yApprox;
    protected Trajectory m_trajectory;
    public static final double ORDER_FACTOR = 12;
    
    public PolynomialCurveApproximation(List<State> states) {
        super(states);
        m_trajectory = new Trajectory(states);
    }

    @Override
    public List<State> getBetterStates() {
        m_xApprox = getXPolynomialApproximation();
        m_yApprox = getYPolynomialApproximation();
        
        Logging.compareGeneratedPolynomialForY("PolynomialY.csv", m_states, m_yApprox);
        Logging.compareGeneratedPolynomialForX("PolynomialX.csv", m_states, m_xApprox);

        List<State> betterStates = generateBetterStates();
        return betterStates;
    }

    private PolynomialFunction getYPolynomialApproximation() {
        ValueGetter yGetter = new ValueGetter() {

			@Override
			public double get(double time) {
				return m_trajectory.sample(time).poseMeters.getY();
			}

        };
        UnivariateFunction yValues = new UnivariateFunction() {

			@Override
			public double value(double t) {
				return m_trajectory.sample(t).poseMeters.getY();
			}

        };
        int numberOfDirectionChanges = getNumberOfDirectionChanges(yValues);
        return getPolynomialApproximation(yGetter, (int)ORDER_FACTOR*(numberOfDirectionChanges + 3), false);
    }

    private PolynomialFunction getXPolynomialApproximation() {
        ValueGetter xGetter = new ValueGetter() {

			@Override
			public double get(double time) {
				return m_trajectory.sample(time).poseMeters.getX();
			}

        };
        UnivariateFunction xValues = new UnivariateFunction() {

			@Override
			public double value(double t) {
				return m_trajectory.sample(t).poseMeters.getX();
			}

        };
        int numberOfDirectionChanges = getNumberOfDirectionChanges(xValues);
        return getPolynomialApproximation(xGetter, (int)ORDER_FACTOR*(numberOfDirectionChanges + 3), true);
    }
    
    protected PolynomialFunction getPolynomialApproximation(ValueGetter valueGetter, int order, boolean debugMode) {
        final WeightedObservedPoints obs = new WeightedObservedPoints();
        for(double time = 0; time < m_trajectory.getTotalTimeSeconds(); time+=0.02) {
            obs.add(time, valueGetter.get(time));
        }

        final PolynomialCurveFitter fitter = PolynomialCurveFitter.create(order);

        // Retrieve fitted parameters (coefficients of the polynomial function).
        final double[] coeff = fitter.fit(obs.toList());
                
        return new PolynomialFunction(coeff);
    }

    private int getNumberOfDirectionChanges(UnivariateFunction values){
        
        FiniteDifferencesDifferentiator differentiator = new FiniteDifferencesDifferentiator(3, 0.1, 0, m_trajectory.getTotalTimeSeconds());
        UnivariateDifferentiableFunction completeX = differentiator.differentiate(values);
        DerivativeStructure derivativeStructure = new DerivativeStructure(1, 1, 0, 0);
        double previousDerivative = 0;
        int numberOfDirectionChanged = 0;
        for(double t = 0.02; t < m_trajectory.getTotalTimeSeconds(); t+=0.02) {
            derivativeStructure = new DerivativeStructure(1, 1, 0, t);
            double currentDerivative  = completeX.value(derivativeStructure).getPartialDerivative(1);
            if(currentDerivative * previousDerivative < 0 ) {
                numberOfDirectionChanged++;
            }
            previousDerivative = currentDerivative;
        }

        return numberOfDirectionChanged;
    }

    abstract class ValueGetter {
        public abstract double get(double time);
    }

    protected abstract List<State> generateBetterStates();
}
