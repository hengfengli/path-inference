package pathinfer.map;

import java.util.*;

/**
 *  This class provide all the methods to access the properties of a node. 
 *  
 *  @author Hengfeng Li
 */
public class OSMNode{
	private String nodeID;
	private LonLat lonlat;
	private int index;
	
	public OSMNode(String nodeID, LonLat lonlat) {
		this.nodeID = nodeID;
		this.lonlat = lonlat;
		this.index  = -1;
	}
	
	// Access attributes
	public String getNodeID()          { return this.nodeID; }
	public LonLat getLonLat()          { return this.lonlat; }
	public int getIndex()              { return this.index;  }
	public void setIndex(int index)    { this.index = index; }
}
