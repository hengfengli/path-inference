package pathinfer.map;

/**
 *  This class provide all the methods to access a boundary. 
 *  
 *  @author Hengfeng Li
 */
public class Boundary {
    private double minX, maxX, minY, maxY;
    private double width, height;
	    
    public Boundary(double minLon, double maxLon, double minLat, double maxLat) {
        
        this.minX = minLon;
        this.maxX = maxLon;
        this.minY = minLat;
        this.maxY = maxLat;
        
        double diffX = maxX-minX;
        double diffY = maxY-minY;
        
        this.width  = Math.abs(diffX);
        this.height = Math.abs(diffY);
    }
	
	/**
	 *  Check whether a point is inside this rectangle. 
	 */
	public boolean isInside(LonLat l) {
        return l.getLon() >= this.minX
            && l.getLon() <= this.maxX
            && l.getLat() >= this.minY
            && l.getLat() <= this.maxY;
	}
	
	// Access attributes
    public double minX() { return this.minX; }
    public double maxX() { return this.maxX; }
    public double minY() { return this.minY; }
    public double maxY() { return this.maxY; }
    public double width(){ return this.width; }
    public double height() { return this.height; }
    
    public String toString() {
        return String.format("%.6f %.6f %.6f %.6f", minX, maxX, minY, maxY);
    }
}
