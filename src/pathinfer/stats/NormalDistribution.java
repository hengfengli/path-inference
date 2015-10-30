package pathinfer.stats;

/**
 *  This class provides all the methods to compute the probability under 
 *  normal distribution.  
 * 
 *  @author Hengfeng Li
 */
public class NormalDistribution {
	private double mu;
	private double theta;
	private double c1;
	private double c2;
	
	public NormalDistribution(double mu, double theta) {
		this.mu = mu;
		this.theta = theta;
		this.c1 = 1/(theta * Math.sqrt(2*Math.PI));
		this.c2 = -0.5;
	}
	
	public double compute(double x) {
		return c1 * Math.exp(c2 * Math.pow((x - mu)/theta, 2));
	}
	
	public double mean() { return this.mu; }
	public double std()  { return this.theta; }
	
	public static void main(String [] args) {
		double DIST_NOISY = 50;
		NormalDistribution norm1 = new NormalDistribution(0, DIST_NOISY);
		System.out.println(norm1.compute(0));
	}
}