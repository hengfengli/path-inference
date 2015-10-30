package pathinfer.map;

import java.util.List;

/**
 *  This class represent a GPS point, which helps for path inference
 *  purpose. It has several candidate edges from which a correct 
 *  mapping edge will be chosen. 
 * 
 *  @author Hengfeng Li
 */
public class GPSPoint {
	private LonLat lonlat;
	private long time;
	private List<Candidate> candidates;
	private String info;
	
	private double weight;
	
	public GPSPoint(LonLat lonlat, long time, String info) {
		this.lonlat = lonlat;
		this.time = time;
		this.info = info;
		this.candidates = null;
		this.weight = -1;
	}
	
	public void setCandidates(List<Candidate> candidates) {
		this.candidates = candidates;
	}
	
	public void setWeight(double weight)   { this.weight = weight; }
	public double getWeight()			   { return this.weight;   }
	public LonLat getLonLat()              { return this.lonlat; }
	public double getLon()                 { return this.lonlat.getLon(); }
	public double getLat()                 { return this.lonlat.getLat(); }
	public long getTime()				   { return this.time; }
	public List<Candidate> getCandidates() { return this.candidates; }
	
	public String toString() {
		String[] params = this.info.split(" ");
					
		StringBuilder str = new StringBuilder();
		
		str.append(params[0]);str.append(" ");  // traj ID
		str.append(params[1]);str.append(" ");  // car ID
		str.append(params[2]);str.append(" ");  // time
		str.append(String.format("%.6f", lonlat.getLat()));str.append(" ");
		str.append(String.format("%.6f", lonlat.getLon()));str.append(" ");
		str.append(params[6]); // speed
		
		return str.toString();
	}
}