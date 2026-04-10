package lib.woodsonrobotics.math;

/**
 * A simple live linear regression that accumulates data points and
 * computes the best-fit line y = slope * x + intercept.
 */
public class LiveRegression {
    private int n = 0;
    private double sumX = 0;
    private double sumY = 0;
    private double sumXY = 0;
    private double sumX2 = 0;

    /** Add a data point (x, y) to the regression. */
    public void addPoint(double x, double y) {
        n++;
        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumX2 += x * x;
    }

    /** Number of data points added so far. */
    public int size() {
        return n;
    }

    /** Whether we have enough points for a regression (at least 2). */
    public boolean isReady() {
        return n >= 2 && (n * sumX2 - sumX * sumX) != 0;
    }

    /** The slope of the best-fit line. */
    public double slope() {
        double denom = n * sumX2 - sumX * sumX;
        return (n * sumXY - sumX * sumY) / denom;
    }

    /** The y-intercept of the best-fit line. */
    public double intercept() {
        return (sumY - slope() * sumX) / n;
    }

    /**
     * Predict y for a given x using the current regression.
     * If fewer than 2 points, this raises an exception.
     */
    public double predict(double x) {
        if (!isReady()) {
            throw new RuntimeException("regression model is not ready");
        }
        return slope() * x + intercept();
    }

    @Override
    public String toString() {
        if (!isReady()) {
            return String.format("LiveRegression(n=%d, need 2+ points)", n);
        }
        return String.format("LiveRegression(n=%d, y=%.2f*x+%.2f)",
                n, slope(), intercept());
    }
}