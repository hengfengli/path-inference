package pathinfer.simp;

import pathinfer.map.GPSPoint;
import pathinfer.map.OSMNode;
import pathinfer.map.LonLat;
import pathinfer.geo.*;
import pathinfer.map.Candidate;
import pathinfer.obr.OrientedBoundingRectangle;
import pathinfer.graph.*;

import java.util.List;
import java.util.ArrayList;

/**
 *  This class provides two simplification algorithms. 
 * 
 *  @author Hengfeng Li
 */
public class Simplification {
    
    /**
     *  This method removes points that are far away from the road network. 
     */
    public static List<GPSPoint> rectFilter(List<GPSPoint> pointList, double epsilon) {
        
        System.out.println("filter old: " + pointList.size());
        
        List<GPSPoint> newTraj = new ArrayList<GPSPoint>();
        
        for (int i = 0; i < pointList.size(); i++) {
            GPSPoint p = pointList.get(i);
            double minDist = Double.MAX_VALUE;
            
            for (Candidate cand: p.getCandidates()) {
                double dist = cand.getDistance();
                if (dist < minDist) { 
                    minDist = dist; 
                }
            }
            
            if (minDist < epsilon) { 
                newTraj.add(p); 
            }
        }
        
        System.out.println("filter new: " + newTraj.size());
        
        return newTraj;
    }
	
    /**
     *  This method is the famous Douglas Peucker algorithm . 
     */
	public static List<GPSPoint> douglasPeucker(List<GPSPoint> pointList, double epsilon) {
		
	    // Find the point with the maximum distance
	    int sizePoints = pointList.size();
	    double dmax = 0, d = 0;
	    int index = 0;
		
	    // Calculate the distances between inside points and the start-end line
		for (int i = 1; i < sizePoints-1; i++) {
			
			d = GeoCalculator.distancePointToLineOnMeters(
				pointList.get(i).getLonLat(), 
				pointList.get(0).getLonLat(), 
				pointList.get(sizePoints-1).getLonLat()				
			);
			if (d > dmax) {
				index = i;
				dmax = d;
			}
		}
		
		List<GPSPoint> resultList = new ArrayList<GPSPoint>();
        
	    // If max distance is greater than epsilon, recursively simplify
	    if (dmax > epsilon) {
			List<GPSPoint> recResults1 = douglasPeucker(pointList.subList(0,index+1), epsilon);
			List<GPSPoint> recResults2 = douglasPeucker(pointList.subList(index,sizePoints), epsilon);
	    	
			resultList.addAll(recResults1.subList(0,recResults1.size()-1));
			resultList.addAll(recResults2);
		} else {
			resultList.add(pointList.get(0));
			resultList.add(pointList.get(sizePoints-1));
		}
        
	    return resultList;
	}
}