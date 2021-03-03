package frc.robot.BetterTrajectoryGenerators;

import java.util.ArrayList;
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

    private Trajectory m_trajectory;
    protected PolynomialFunction m_xApprox;
    protected PolynomialFunction m_yApprox;
    public PolynomialCurveApproximation(List<State> states) {
        super(states);
        m_trajectory = new Trajectory(m_states);
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
			public double get(int index) {
				return m_states.get(index).poseMeters.getY();
			}

        };
        UnivariateFunction yValues = new UnivariateFunction() {

			@Override
			public double value(double t) {
				// TODO Auto-generated method stub
				return m_trajectory.sample(t).poseMeters.getY();
			}

        };
        int numberOfDirectionChanges = getNumberOfDirectionChanges(yValues);
        return getPolynomialApproximation(yGetter, 2*(numberOfDirectionChanges + 3), false);
    }

    private PolynomialFunction getXPolynomialApproximation() {
        ValueGetter xGetter = new ValueGetter() {

			@Override
			public double get(int index) {
				return m_states.get(index).poseMeters.getX();
			}

        };
        UnivariateFunction xValues = new UnivariateFunction() {

			@Override
			public double value(double t) {
				// TODO Auto-generated method stub
				return m_trajectory.sample(t).poseMeters.getX();
			}

        };
        int numberOfDirectionChanges = getNumberOfDirectionChanges(xValues);
        return getPolynomialApproximation(xGetter, 2*(numberOfDirectionChanges + 3), true);
    }

    private PolynomialFunction getPolynomialApproximation(ValueGetter valueGetter, int order, boolean debugMode) {

        int orderOfPolynomialApproximation = order; 
        if(debugMode) {
            System.out.println(order);
        }
        double[] coefficients = new double[orderOfPolynomialApproximation + 1];
        coefficients[0] = valueGetter.get(0);
        coefficients[1] = 0;

        int orderOfPenUltimateThetaFunction = orderOfPolynomialApproximation - 3;
        double[] penUltimateThetaCoefficients = new double[orderOfPenUltimateThetaFunction + 1];
        final double deltaY = valueGetter.get(m_states.size() - 1) - valueGetter.get(0);
        final double finalTime = m_states.get(m_states.size() - 1).timeSeconds;
        double denominator = (1.0 - (orderOfPolynomialApproximation - 1.0) / orderOfPolynomialApproximation) * Math.pow(finalTime, orderOfPolynomialApproximation - 1);
        
        penUltimateThetaCoefficients[0] = deltaY / denominator;
        for(int i = 1; i < penUltimateThetaCoefficients.length; i++) {
            penUltimateThetaCoefficients[i] = ((i+1.0)/(orderOfPolynomialApproximation) - 1.0) * Math.pow(finalTime, i+1.0) / denominator;
        }

        int orderOfFinalThetaFunction = orderOfPenUltimateThetaFunction;
        double[] finalThetaCoefficients = new double[orderOfFinalThetaFunction + 1];
        denominator = (1 - orderOfPolynomialApproximation / (orderOfPolynomialApproximation - 1.0)) * Math.pow(finalTime, orderOfPolynomialApproximation);
        finalThetaCoefficients[0] = deltaY / denominator;

        for(int i = 1; i < penUltimateThetaCoefficients.length; i++) {
            finalThetaCoefficients[i] = ((i+1.0)/(orderOfPolynomialApproximation - 1.0) - 1.0) * Math.pow(finalTime, i+1.0) / denominator;
        }

        double[] outputData = new double[m_states.size()];
        for(int i = 0; i < outputData.length; i++) {
            outputData[i] = valueGetter.get(i) - coefficients[0] - penUltimateThetaCoefficients[0] - finalThetaCoefficients[0];
        }

        WeightedObservedPoints obs = new WeightedObservedPoints();
        int count = 0;
        for(State state : m_states) {
            obs.add(Math.pow(state.timeSeconds, 2), outputData[count]);
            count++;
        }
        PolynomialCurveFitter fitter = PolynomialCurveFitter.create(orderOfPolynomialApproximation - 4);
        double[] coeffForMiddleParameters = fitter.fit(obs.toList());
        count = 2;
        for(double coeff : coeffForMiddleParameters) {
            coefficients[count] = coeff;
            count++;
        }

        double penUltimateCoefficient = penUltimateThetaCoefficients[0];
        for(int i = 0; i < coeffForMiddleParameters.length; i++) {
            penUltimateCoefficient += penUltimateThetaCoefficients[i+1] * coeffForMiddleParameters[i];
        }

        double finalCoefficient = finalThetaCoefficients[0];
        for(int i = 0; i < coeffForMiddleParameters.length; i++) {

            finalCoefficient += finalThetaCoefficients[i+1] * coeffForMiddleParameters[i];
        }

        coefficients[coefficients.length - 2] = penUltimateCoefficient;
        coefficients[coefficients.length - 1] = finalCoefficient;

        if(debugMode){
            prettPrint(coefficients);
        }
        return new PolynomialFunction(coefficients);
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

    private void prettPrint(double[] arr) {
        for(double x : arr) {
            System.out.println(x);
        }
    }

    private abstract class ValueGetter {
        public abstract double get(int index);
    }

    protected abstract List<State> generateBetterStates();
    
}
