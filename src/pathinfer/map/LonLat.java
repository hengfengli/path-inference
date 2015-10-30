package pathinfer.map;

/**
 *  This class provide all the methods to access a point in geography. 
 *  
 *  @author Hengfeng Li
 */
public class LonLat {
	private double lon;
	private double lat;
	
	public LonLat(double lon, double lat) {
		this.lon = lon;
		this.lat = lat;
	}
	
	// Access attributes
	public double getLon()           { return lon; }
	public void   setLon(double lon) { this.lon = lon; }
	public double getLat()           { return lat; }
	public void   setLat(double lat) { this.lat = lat; }
	public String toString()         {
		return String.format("%f %f", this.lat, this.lon); 
	}
}