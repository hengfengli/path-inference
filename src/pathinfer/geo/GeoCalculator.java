package pathinfer.geo;

import pathinfer.map.*;
import pathinfer.main.Constant;

import java.awt.geom.Rectangle2D;
import java.awt.geom.Line2D;

import edu.princeton.cs.algs4.Point2D;


/**
 *  This class provides all methods of computing spatial distance and turning 
 *  angles. 
 *  
 *  Some part of codes from Geodesy by Mike Gavaghan
 *  http://www.gavaghan.org/blog/free-source-code/geodesy-library-vincentys-formula/
 *  
 */
public class GeoCalculator {
	
    /**
      * Detect whether a line intersects with a rectangle. 
      */
	public static boolean isLineIntersectRect(
		double x1, double y1, double x2, double y2,
		double rectX, double rectY, double width, double height) {
		
		Rectangle2D rect = new Rectangle2D.Double(rectX, rectY, width, height);
		Line2D line = new Line2D.Double(x1, y1, x2, y2);
		
		return line.intersects(rect);
	}
	
    /**
      * Calculate the distance (meters) between two geometric points. 
      */
	public static double calculateDistance(LonLat n1, LonLat n2) {
		
		double theta = n1.getLon() - n2.getLon();
		double theta_lat = n1.getLat() - n2.getLat();
		
		if ((n1.getLon() == n2.getLon() && n1.getLat() == n2.getLat())
		 || (Math.abs(theta) < Constant.EPSILON && Math.abs(theta_lat) < Constant.EPSILON))
		{
			return 0.0;
		}
				
		double radians_theta  = Math.toRadians(theta);
		double radians_n1_lat = Math.toRadians(n1.getLat());
		double radians_n2_lat = Math.toRadians(n2.getLat());
		
		double dist = (
			    Math.sin(radians_n1_lat)
			  * Math.sin(radians_n2_lat))
			  + (Math.cos(radians_n1_lat)
				   * Math.cos(radians_n2_lat)
				   * Math.cos(radians_theta)
				);
		dist = Math.acos(dist);
		dist = Math.toDegrees(dist);
		dist = dist * 60 * 1.1515;
		dist = dist * 1609.344;
		return dist;
	}
	
    /**
      * Calculate the Euclidean distance between two points. 
      */
	public static double euclideanDistance(LonLat n1, LonLat n2) {
		return Math.sqrt(
			Math.pow(n1.getLon() - n2.getLon(),2) 
		  + Math.pow(n1.getLat() - n2.getLat(),2)
		);
	}
	
    /**
      * Calculate the distance (meters) from a point to a line.  
      */
	public static double distancePointToLineOnMeters(
		LonLat C, LonLat A, LonLat B) {
		
		LonLat point = pointProjectionOnLineIgnoreInsideOrNot(C, A, B);
		
		return calculateDistance(C, point);
	}
    
    /**
      * Calculate the Euclidean distance from a point to a line.  
      */
	public static double distancePointToLineOnEuclideanDist(
		LonLat C, LonLat A, LonLat B) {
		
		LonLat point = pointProjectionOnLineIgnoreInsideOrNot(C, A, B);
		
		return euclideanDistance(C, point);
	}
    
    /**
      * Calculate the distance (meters) from a point to a line, but the 
      * projection of that point must be on the line. 
      */
	public static double distancePointToProjectedLine(
		LonLat C, LonLat A, LonLat B) {
		
		LonLat point = pointProjectionOnLine(C, A, B);
		
		return calculateDistance(C, point);
	}
    
    /**
      * Return the projection point on the line (even this point is not 
	  * on this line). If this point is not on the line, then compare 
	  * its distance to start node or end node to see which is more 
	  * close, then map it. 
      */
	public static LonLat pointProjectionOnLine(LonLat C,
		LonLat A, LonLat B) {
		double Cx = C.getLon(), Cy = C.getLat();
		double Ax = A.getLon(), Ay = A.getLat();
		double Bx = B.getLon(), By = B.getLat();
		
		double L_2 = Math.pow(Bx-Ax, 2) + Math.pow(By-Ay, 2);

		double r = ((Ay-Cy)*(Ay-By) - (Ax-Cx)*(Bx-Ax))/ L_2;
		
		double newX = Ax + r*(Bx-Ax);
		double newY = Ay + r*(By-Ay);
		LonLat newPoint = new LonLat(newX, newY);
		
		if (!isInLineBounds(newPoint, A, B)) {
			// If this projection node is not in the line,
			// then compute the minimum of distances start node 
			// and end node. 
			double distToStart = euclideanDistance(newPoint, A);
			double distToEnd   = euclideanDistance(newPoint, B);
			
			if (distToStart < distToEnd) newPoint = new LonLat(Ax,Ay);
			else                         newPoint = new LonLat(Bx,By);
		}
		
		return newPoint;
	}
	
    /**
      * Calculate the projected point on a line. 
      */
	public static LonLat pointProjectionOnLineIgnoreInsideOrNot(LonLat C,
		LonLat A, LonLat B) {
		double Cx = C.getLon(), Cy = C.getLat();
		double Ax = A.getLon(), Ay = A.getLat();
		double Bx = B.getLon(), By = B.getLat();
		
		double L_2 = Math.pow(Bx-Ax, 2) + Math.pow(By-Ay, 2);

		double r = ((Ay-Cy)*(Ay-By) - (Ax-Cx)*(Bx-Ax))/ L_2;
		
		double newX = Ax + r*(Bx-Ax);
		double newY = Ay + r*(By-Ay);
		LonLat newPoint = new LonLat(newX, newY);
		
		return newPoint;
	}

    /**
      * Return true if C is inside the rectangle of A and B. 
      */
	public static boolean isInLineBounds(
		LonLat C, LonLat A, LonLat B) {
		
		double min_lon = Math.min(A.getLon(), B.getLon());
		double min_lat = Math.min(A.getLat(), B.getLat());
		double max_lon = Math.max(A.getLon(), B.getLon());
		double max_lat = Math.max(A.getLat(), B.getLat());
		
		Boundary boundary = new Boundary(min_lon, max_lon, min_lat, max_lat);
		
		return boundary.isInside(C);
	}
	
    /**
      * Calculate the angle of a vector, which starts from 'a' to 'b'.
      */
    public static double calcAngle(double aX, double aY, double bX, double bY) {
	    double angle = (int)Math.toDegrees(
			Math.atan2(bY - aY, bX - aX));
	    
		if(angle < 0) angle += 360;
	    return angle;
    }
    
    /**
      * Calculate the angle for two LonLat points. 
      */
	public static double calcAngle(LonLat a, LonLat b) {
        return calcAngle(a.getLon(), a.getLat(), b.getLon(), b.getLat());
	}
    
    /**
      * Calculate the angle for two Point2D points. 
      */
    public static double calcAngle(Point2D a, Point2D b) {
        return calcAngle(a.x(), a.y(), b.x(), b.y());
    }
    
    /**
      * Calculate the turning angle ranging from 0 to 180 degrees.
      */
	public static double calcAngle180(LonLat a, LonLat b) {
	    return diffAngle180(calcAngle(a,b), 0);
	}
    
    /**
      * Compute the difference of two degrees (0-180).
      */	
    public static double diffAngle180(double a, double b) {
        double c = Math.abs(a - b);
        return Math.min(c, 360 - c);
    }
	
    /**
      * Compute the turning angle.
      */
    public static double diffTurningAngle(double a, double b) {
        double c = a - b;
        c = (c + 180) % 360 - 180;
        return Math.abs(c);
    }
}