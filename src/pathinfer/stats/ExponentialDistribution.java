package pathinfer.stats;

/**
 *  This class provides all the methods to compute the probability under 
 *  an exponential distribution.  
 *  
 *  @author Hengfeng Li
 */
public class ExponentialDistribution{
	private double alpha;
	private double beta;
	
	public ExponentialDistribution(double alpha, double beta) {
		this.beta  = beta;
		this.alpha = alpha;
	}
    
	public double compute(double dt) {
		// return (1/beta) * Math.exp(-(dt/beta));
		return alpha * Math.exp(-(dt/beta));
	}
}