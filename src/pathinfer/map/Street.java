package pathinfer.map;

import pathinfer.geo.GeoCalculator;

/**
 *  This class provide all the methods to access the properties of a street.
 *  
 *  @author Hengfeng Li
 */
public class Street {
	private int streetID;
	private OSMNode startNode;
	private OSMNode endNode;
	private double distance;
	private int level;         // types of street, like motorway, residential
	private int angle;         // angle of street
	
	public Street(int streetID, OSMNode startNode, OSMNode endNode, 
		double distance, int level) {
		this.startNode  = startNode;
		this.endNode    = endNode;
		this.distance   = distance;
		this.streetID   = streetID;
		this.level      = level;
		this.angle      = (int)GeoCalculator.calcAngle(startNode.getLonLat(), 
			                                           endNode.getLonLat());
	}
	
	// Access attributes
	public double getDistance()    { return this.distance; }
	public OSMNode getStartNode()  { return this.startNode;}
	public OSMNode getEndNode()    { return this.endNode;  }
	public int getStreetID()       { return this.streetID; }
    public String getStartNodeID() { return this.startNode.getNodeID(); }
    public String getEndNodeID()   { return this.endNode.getNodeID();   }
	public int getLevel()          { return this.level; }
	public int getAngle()          { return this.angle; }
}