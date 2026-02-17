package frc.robot.utils;

import java.util.ArrayList;

import javax.tools.Diagnostic;

import org.apache.commons.math3.ode.FirstOrderDifferentialEquations;
import org.apache.commons.math3.ode.FirstOrderIntegrator;
import org.apache.commons.math3.ode.events.EventHandler;
import org.apache.commons.math3.ode.nonstiff.DormandPrince853Integrator;
import org.apache.commons.math3.ode.nonstiff.EulerIntegrator;
import org.apache.commons.math3.ode.sampling.StepHandler;
import org.apache.commons.math3.ode.sampling.StepInterpolator;
import org.dyn4j.world.Island;

import com.ctre.phoenix6.signals.DiffPIDOutput_PIDOutputModeValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

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
    // Ball weight (N)
    private static final double weight = mass * g;
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

    public static final double mu = 0.5 * dragCoeff * rho * area;

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

    /**
     * Check whether a position's x value is within some distance of a target
     * position
     * 
     * @param currentPosition The current position
     * @param targetPosition  The target position
     * @param accuracy        The acceptable error distance
     * @return
     */
    private static boolean checkResult(Translation2d currentPosition, Translation2d targetPosition, double accuracy) {
        return currentPosition.getX() <= targetPosition.getX() + accuracy
                && currentPosition.getX() >= targetPosition.getX() - accuracy;
    }

    /// **
    // * A quick function that runs a ballistic simulation
    // *
    // * @deprecated
    // * @param angle
    // * @param guess
    // * @param targetPosition
    // * @return
    // */
    // private static Translation2d runSim(double angle, double guess, Translation2d
    // targetPosition){
    // return simulate(new Translation2d(0, 0), polarToCartesian(guess, angle),
    // targetPosition.getY(), Y, 0.01).endPos;
    // }

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
    // Possible end conditions
    // public static final int TIME = 0;
    // public static final int X = 1;
    // public static final int Y = 2;

    /// **
    // * Runs the ballistics simulation (UNFINISHED)
    // *
    // * @deprecated
    // * @param startPos The starting position of the ball
    // * @param startVel The starting velocity of the ball
    // * @param endVal The value that will be used for the end condition
    // * @param endCond The condition that will trigger the simulation to end
    // * @param stepSize The timestep of the simulation in seconds
    // * @return
    // */
    // public static BallisticsSimResult simulate(
    // Translation2d startPos,
    // Translation2d startVel,
    // double endVal,
    // int endCond,
    // double stepSize
    // ){
    // Translation2d pos = startPos;
    // Translation2d velocity = startVel;
    // ArrayList<Translation2d> path = new ArrayList<Translation2d>();
    // path.add(pos);
    // double time = 0;
    // double velocityMult = stepSize/mass;
    // double gravityAcceleration = g * stepSize;
    // boolean simulationFinished = false;
    // while(!simulationFinished){
    // switch (endCond) {
    // case 0:
    // simulationFinished = time >= endVal;
    // break;
    // case 1:
    // simulationFinished = pos.getX() >= endVal;
    // break;
    // case 2:
    // simulationFinished = pos.getY() <= endVal && velocity.getY() < 0;
    // break;
    // default:
    // throw new Error(String.format("%d is not a valid end condition", endCond));
    //
    // }
    //
    // // Apply drag
    // velocity = velocity.minus(new Translation2d(dragForce(velocity.getX()),
    // dragForce(velocity.getY())).times(velocityMult));
    // // Apply gravity
    // velocity = velocity.minus(new Translation2d(0, gravityAcceleration));
    // // Update position
    // pos = pos.plus(velocity.times(stepSize));
    // // Add current position to path
    // path.add(pos);
    // // Update time
    // time += stepSize;
    // }
    // return new BallisticsSimResult(path, pos, velocity, time);
    // }

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

    public static BallisticsSimResult ODESimulate(Translation2d startPos, Translation2d startVel, double targetY) {
        endReached = true;
        endTime = 0;
        // The event handler that will end the simulation when the ball reaches the
        // height of the target for the second time (the first time it will be going
        // upwards, the second time it will be going downwards)
        EventHandler mainEventHandler = new EventHandler() {
            // The number of times targetY has been crossed
            private int numCrossed = 0;

            public void init(double t0, double[] y0, double t) {
            }

            public double g(double t, double[] y) {
                return (y[1] - targetY);
            }

            public EventHandler.Action eventOccurred(double t, double[] y, boolean increasing) {
                if (numCrossed == 0) {
                    numCrossed = 1;
                    return (EventHandler.Action.CONTINUE);
                } else {
                    return (EventHandler.Action.STOP);
                }
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

    ///**
    // * Determines the speed required to send a ball through a certain position
    // * (UNFINISHED)
    // * 
    // * @deprecated
    // * @param position            The position that the ball will be sent through
    // * @param guess               The initial guess for what the speed should be
    // * @param boundSearchInterval
    // * @param accuracy            The accuracy that must be met (in meters)
    // * @param minAccuracy         The minimum acceptable accuracy that will allow
    // *                            the program to return its result if the maximum
    // *                            attempts are exceeded (in meters)
    // * @param angle               The angle the ball is fired at
    // * @param maxAttempts         The maximum number of attempts the program can use
    // *                            to try to find the speed
    // * @return The determined speed (in meters per second)
    // */
    // public static double targetPosition(
    // Translation2d position,
    // double guess,
    // double boundSearchInterval,
    // double accuracy,
    // double minAccuracy,
    // double angle,
    // double maxAttempts
    // ){
    // double currentGuess = guess;
    // Translation2d currentPosition = runSim(angle, currentGuess, position);
    // if(!checkResult(currentPosition, position, accuracy)){
    // // Perform a binary search to find the result
    // // Find the initial boundaries
    // double[] bounds = {0,0};
    // if(currentPosition.getX() < position.getX()){
    // bounds[0] = currentGuess;
    // // Walk forwards to find the first boundary
    // while(currentPosition.getX() < position.getX()){
    // currentGuess += boundSearchInterval;
    // currentPosition = runSim(angle, currentGuess, position);
    // }
    // bounds[1] = currentGuess;
    // }else{
    // bounds[1] = currentGuess;
    // // Walk forwards to find the first boundary
    // while(currentPosition.getX() > position.getX()){
    // currentGuess -= boundSearchInterval;
    // currentPosition = runSim(angle, currentGuess, position);
    // }
    // bounds[0] = currentGuess;
    // }
    // if(checkResult(currentPosition, position, accuracy)){
    // return(currentGuess);
    // }
    // // Perform the binary search
    // int attempts = 0;
    // while(!checkResult(currentPosition, position, accuracy)){
    // // Set the current guess halfway between the upper and lower bounds
    // currentGuess = (bounds[0]+bounds[1])/2;
    // currentPosition = runSim(angle, currentGuess, position);
    // // Set a new upper or lower bounds depending on whether the guess overshot or
    // undershot
    // if(currentPosition.getX() < position.getX()){
    // bounds[0] = currentGuess;
    // }else{
    // bounds[1] = currentGuess;
    // }
    // attempts += 1;
    // if(attempts >= maxAttempts){
    // if(Math.abs(currentPosition.getX() - position.getX()) < minAccuracy){
    // System.out.print(String.format("Targeting exceeded %,d attempts, but was
    // within acceptable accuracy", maxAttempts));
    // return(currentGuess);
    // }else{
    // System.err.print(String.format("Targeting exceeded %,d attempts and has been
    // cancelled \n\tCurrent guess: %.4f \n\tTarget position: (%.2f,%.2f)
    // \n\tCurrent position: (%.2f,%.2f)", maxAttempts, currentGuess,
    // position.getX(), position.getY(), currentPosition.getX(),
    // currentPosition.getY()));
    // return(-1);
    // }
    // }
    // }
    // return(currentGuess);
    // }else{
    // return(currentGuess);
    // }
    // }

    /**
     * A function that can determine the necessary initial velocity to send a
     * projectile through a point in space
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
}