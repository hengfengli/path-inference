package pathinfer.map;

/**
 *  This class represents a candidate which consists of the identity
 *  of a street and the distance from the GPS point to this street. 
 *  
 *  @author Hengfeng Li
 */ 
public class Candidate {
	private int streetID;
	private double distance; // meters
	private Candidate previous;
	private double prob;
	
	public Candidate(int streetID, double distance) {
		this.streetID = streetID;
		this.distance = distance;
		this.previous = null;
		this.prob     = -Double.MAX_VALUE;
	}
	
	public void setProb(double prob) { this.prob = prob; }
	public void setPrev(Candidate candidate) { this.previous = candidate; }
	public double prob()        { return this.prob;     }
	public Candidate prev()     { return this.previous; }
	public int getStreetID()    { return this.streetID; }
	public double getDistance() { return this.distance; }
	public String toString()
	{
		return String.format("%d->%.2f", streetID, distance);
	}
	
	@Override
	public int hashCode() {
	    return streetID;
	}
	
	@Override
	public boolean equals(Object obj) {
	    if (this == obj)
	        return true;
	    if (obj == null)
	        return false;
	    if(obj instanceof Candidate) {
	        Candidate s1 = (Candidate)obj;

	        if(s1.getStreetID() == this.getStreetID()) {
	            return true;
	        }
	    }
	    return false;
	}
}