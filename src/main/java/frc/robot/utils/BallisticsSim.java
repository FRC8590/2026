package frc.robot.utils;

import org.apache.commons.math3.ode.FirstOrderDifferentialEquations;
import org.apache.commons.math3.ode.FirstOrderIntegrator;
import org.apache.commons.math3.ode.events.EventHandler;
import org.apache.commons.math3.ode.nonstiff.DormandPrince853Integrator;
import org.apache.commons.math3.ode.sampling.StepHandler;
import org.apache.commons.math3.ode.sampling.StepInterpolator;

import edu.wpi.first.math.geometry.Translation2d;

public class BallisticsSim {

    // Earth stats
    // Air density (kg/m^3)
    private static final double rho = 1.2250;
    // Gravity (m/s^2)
    private static final double g = 9.8;

    // Ball stats
    // Ball diameter (m)
    private static final double diameter = 0.15;
    // Ball projected area
    private static final double area = Math.pow(diameter / 2, 2) * Math.PI;
    // Ball mass (kg)
    private static final double mass = 0.21;
    // Ball coefficient of drag (unitless)
    private static final double dragCoeff = 0.471;

    // Basic functions
    /**
     * A function to get the drag force on the ball in Newtons for a specific speed
     * 
     * @param speed The speed of the ball
     */
    public static double dragForce(double speed) {
        return (0.5 * dragCoeff * rho * area * Math.pow(speed, 2) * Math.signum(speed));
    }

    // A unitless multiplier for the drag force calculations
    public static final double mu = (0.5 * dragCoeff * rho * area) / mass;

    /**
     * Convert a polar coordinate to a cartesian coordinate
     * 
     * @param radius The radius component of the polar coordinate
     * @param angle  The angle component of the polar coordinate
     * @return The resulting cartesian coordinate
     */
    public static Translation2d polarToCartesian(double radius, double angle) {
        return (new Translation2d(Math.cos(Math.toRadians(angle)) * radius, Math.sin(Math.toRadians(angle)) * radius));
    }

    /**
     * Convert a polar coordinate to a cartesian coordinate
     * 
     * @param coordinate The polar coordinate (x is angle, y is radius)
     * @return The resulting cartesian coordinate
     */
    public static Translation2d polarToCartesian(Translation2d coordinate) {
        return (polarToCartesian(coordinate.getX(), coordinate.getY()));
    }

    public static class BallisticsSimResult {
        public Translation2d endPos;
        public Translation2d endVel;
        public double endTime;
        public boolean endReached;

        public BallisticsSimResult(Translation2d endPos, Translation2d endVel, double endTime, boolean endReached) {
            this.endPos = endPos;
            this.endVel = endVel;
            this.endTime = endTime;
            this.endReached = endReached;
        }

        public BallisticsSimResult(double[] endPos, double[] endVel, double endTime, boolean endReached) {
            this.endPos = new Translation2d(endPos[0], endPos[1]);
            this.endVel = new Translation2d(endVel[0], endVel[1]);
            this.endTime = endTime;
            this.endReached = endReached;
        }

        public BallisticsSimResult(double[] y, double endTime, boolean endReached) {
            this.endPos = new Translation2d(y[0], y[1]);
            this.endVel = new Translation2d(y[2], y[3]);
            this.endTime = endTime;
            this.endReached = endReached;
        }
    }

    private static class BallisticsODE implements FirstOrderDifferentialEquations {
        public int getDimension() {
            return 4;
        }

        public void computeDerivatives(double t, double[] y, double[] yp) {
            // x = y[0], y = y[1], vx = y[2], vy = y[3]
            double coeff = Math.sqrt((y[2] * y[2]) + (y[3] * y[3]));
            // Position
            yp[0] = y[2];
            yp[1] = y[3];
            // Velocity
            yp[2] = -mu * y[2] * coeff;
            yp[3] = -g - (mu * y[3] * coeff);

        }
    }

    // Variables for ODESimulate that need to be in the main class so that they can
    // be accessed from handlers
    private static boolean endReached = true;
    private static double endTime = 0;

    /**
     * Simulate the trajectory of a fuel using Apache Math Common's Ordinary
     * Differential Equation solver
     * 
     * @param startPos The starting position of the ball (M)
     * @param startVel The starting velocity of the ball (M/s)
     * @param targetX  The x position that the ball must reach to end the simulation
     *                 (M)
     * @return
     */
    public static BallisticsSimResult ODESimulate(Translation2d startPos, Translation2d startVel, double targetX) {
        endReached = true;
        endTime = 0;
        // The event handler that will end the integration when the ball passes the X
        // position of the target
        EventHandler mainEventHandler = new EventHandler() {

            public void init(double t0, double[] y0, double t) {
            }

            public double g(double t, double[] y) {
                return (y[0] - targetX);
            }

            public EventHandler.Action eventOccurred(double t, double[] y, boolean increasing) {
                return (EventHandler.Action.STOP);
            }

            public void resetState(double t, double[] y) {
            }
        };
        // The event handler that will end the simulation when the ball passes below y
        // 0. This is just to prevent the integration from runnning forever
        EventHandler backupEventHandler = new EventHandler() {
            public void init(double t0, double[] y0, double t) {
            }

            public double g(double t, double[] y) {
                return (y[1]);
            }

            public EventHandler.Action eventOccurred(double t, double[] y, boolean increasing) {
                endReached = false;
                return Action.STOP;
            }

            public void resetState(double t, double[] y) {
            }
        };

        StepHandler stepHandler = new StepHandler() {
            public void init(double t0, double[] y0, double t) {
            }

            public void handleStep(StepInterpolator interpolator, boolean isLast) {
                if (isLast) {
                    endTime = interpolator.getCurrentTime();
                }
            }
        };
        // I'm using this integrator because it's what's used in all the examples, but
        // if there are any recommendations for other integrators that would be better
        // for this use case, I would appreciate them
        FirstOrderIntegrator integrator = new DormandPrince853Integrator(1.0e-8, 100.0, 1.0e-10, 1.0e-10);
        FirstOrderDifferentialEquations ode = new BallisticsODE();
        integrator.addEventHandler(mainEventHandler, 0.1, 1.0e-6, 1000);
        integrator.addEventHandler(backupEventHandler, 0.1, 1.0e-6, 1000);
        integrator.addStepHandler(stepHandler);
        double[] y = new double[] { startPos.getX(), startPos.getY(), startVel.getX(), startVel.getY() };
        integrator.integrate(ode, 0, y, 100, y);
        return (new BallisticsSimResult(y, endTime, endReached));
    }

    /**
     * A function that can determine the necessary initial velocity to send a
     * projectile through a point in space without drag
     * 
     * @param position The point the projectile will pass through RELATIVE TO THE
     *                 INTITIAL POSITION. The projectile will always start at (0,0)
     * @param angle    The initial angle the projectile will be launched at
     * @return The speed in meters/second required to send the projectile through
     *         the position
     */
    public static double targetPositionNaive(Translation2d position, double angle) {
        double a = angle;
        double x = position.getX();
        double y = position.getY();
        return (x * Math.pow(
                Math.cos(Math.toRadians(a)) *
                        Math.sqrt(-(2 * (y - x * (Math.sin(Math.toRadians(a)) / Math.cos(Math.toRadians(a))))) / g),
                -1));
    }

    public static boolean withinMargin(double center, double margin, double value) {
        return (value <= center + margin && value >= center - margin);
    }

    public static final double shooterAngle = 45;
    public static final double derivativeStep = 0.01;

    /**
     * A function that can determine the necessary initial velocity to send a
     * projectile through a point in space
     * 
     * @param target The target point (M)
     * @param accuracyMargin The margin of accuracy (M)
     * @return The speed required to send the projectile through the point (M/s)
     */
    public static double targetPosition(Translation2d target, double accuracyMargin) {
        if ((target.getY() / target.getX()) >= Math.tan(Math.toRadians(shooterAngle))) {
            System.err.println("Invalid target position");
            return (-1);
        }
        double currentGuess = targetPositionNaive(target, shooterAngle);
        BallisticsSimResult currentResult = ODESimulate(new Translation2d(0, 0),
                polarToCartesian(currentGuess, shooterAngle), target.getX());
        if (withinMargin(target.getY(), accuracyMargin, currentResult.endPos.getY())) {
            return (currentGuess);
        } else {
            while (!withinMargin(target.getY(), accuracyMargin, currentResult.endPos.getY())) {
                // Use Newton's method to find the correct speed
                BallisticsSimResult slopeResult = ODESimulate(new Translation2d(0, 0),
                        polarToCartesian(currentGuess + derivativeStep, shooterAngle), target.getX());
                double slope = (slopeResult.endPos.getY() - currentResult.endPos.getY()) / derivativeStep;
                currentGuess = currentGuess - (currentResult.endPos.getY() - target.getY()) / slope;
                currentResult = ODESimulate(new Translation2d(0, 0), polarToCartesian(currentGuess, shooterAngle),
                        target.getX());
                double currentError = currentResult.endPos.getY() - target.getY();
                System.out.println(currentError);
            }
            return (currentGuess);
        }
    }
}
