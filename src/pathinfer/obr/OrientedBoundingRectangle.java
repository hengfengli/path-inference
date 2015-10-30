package pathinfer.obr;

import java.util.*;
import java.io.*;

import pathinfer.map.*;
import pathinfer.geo.*;
import pathinfer.graph.*;
import pathinfer.stats.NormalDistribution;
import pathinfer.stats.ExponentialDistribution;
import pathinfer.main.Constant;

import edu.princeton.cs.algs4.Point2D;

/**
 *  This class implements the Oriented Bounding Rectangle algorithm.
 *  
 *  @author Hengfeng Li
 */
public class OrientedBoundingRectangle {
    private EdgeWeightedDigraph G;
    private List<OSMNode> nodes;
    private List<Street> streets;
    
    private String outFilename;
    private DijkstraSP sp;
    private boolean useRect;
    
    private final NormalDistribution norm;
    private final ExponentialDistribution exp;
    
    /**
      * Constructor. 
      */
    public OrientedBoundingRectangle(EdgeWeightedDigraph G, List<OSMNode> nodes, List<Street> streets, 
        String outFilename, DijkstraSP sp, boolean useRect) {
        this.G = G;
        this.nodes = nodes;
        this.streets = streets;
        this.outFilename = outFilename;
        this.sp = sp;
        
        this.useRect = useRect;
        
        this.norm = new NormalDistribution(0, Constant.NOISE_SIGMA);
        this.exp = new ExponentialDistribution(Constant.EXP_ALPHA, Constant.EXP_BETA);
    }
    
    /**
      * Compute the density of a trajectory. 
      */
    public double computeDensity(List<GPSPoint> traj) {
        
        double sumDistances = 0.;
        
        for (int i = 1; i < traj.size(); i++) {
            GPSPoint prev = traj.get(i-1);
            GPSPoint curr = traj.get(i);
            
            sumDistances += GeoCalculator.calculateDistance(prev.getLonLat(), curr.getLonLat());
        }
        
        return 1/(sumDistances/(traj.size()-1));
    }
    
    /**
      * Calculate the minimum bounding box of a set of points. 
      */
    public Boundary minBoundingBox(List<GPSPoint> points) {
        
        Point2D[] pts = new Point2D[points.size()];
        double minX = points.get(0).getLonLat().getLon();
        double maxX = minX;
        double minY = points.get(0).getLonLat().getLat();
        double maxY = minY;
        
        for (int i = 0; i < points.size(); i++) {
            pts[i] = new Point2D(
                points.get(i).getLonLat().getLon(), 
                points.get(i).getLonLat().getLat()
            );
            
            if (pts[i].x() < minX) minX = pts[i].x();
            if (pts[i].x() > maxX) maxX = pts[i].x();
            if (pts[i].y() < minY) minY = pts[i].y();
            if (pts[i].y() > maxY) maxY = pts[i].y();
        }
        
        return new Boundary(minX, maxX, minY, maxY);
    }
    
    /**
      * Compute the average perpendicular distance from points to the line. 
      */
    public double avgDistToAxis(List<GPSPoint> tempPoints, List<Point2D> line) {
        
        double totalDist = 0.;
        Point2D from = line.get(0);
        Point2D to   = line.get(1);
        
        for (GPSPoint p: tempPoints) {
            LonLat C = p.getLonLat();
            
            LonLat A = new LonLat(from.x(), from.y());
            LonLat B = new LonLat(to.x(), to.y());
            
            LonLat projection = GeoCalculator.pointProjectionOnLineIgnoreInsideOrNot(C, A, B);
            totalDist += GeoCalculator.euclideanDistance(C, projection);
        }
        
        return totalDist/tempPoints.size();
    }
    
    /**
      * Find out the most fit axis from four possible axes of a rectangle. 
      */
    public List<Point2D> approxDirectedLineSegment(List<GPSPoint> tempPoints, Boundary minBBox) {
        double middleX = (minBBox.minX()+minBBox.maxX())*0.5;
        double middleY = (minBBox.minY()+minBBox.maxY())*0.5;
        
        List<Point2D> axes = new ArrayList<Point2D>(8);
        
        axes.add(new Point2D(minBBox.minX(), minBBox.minY()));
        axes.add(new Point2D(minBBox.maxX(), minBBox.maxY()));

        axes.add(new Point2D(minBBox.minX(), minBBox.maxY()));
        axes.add(new Point2D(minBBox.maxX(), minBBox.minY()));
        
        axes.add(new Point2D(middleX, minBBox.minY()));
        axes.add(new Point2D(middleX, minBBox.maxY()));
        
        axes.add(new Point2D(minBBox.minX(), middleY));
        axes.add(new Point2D(minBBox.maxX(), middleY));
        
        double minAvgDist = Double.MAX_VALUE;
        List<Point2D> minLine = null;
        for (int i = 0; i < axes.size(); i+=2) {
            List<Point2D> line = axes.subList(i, i+2);
            double avgDist = avgDistToAxis(tempPoints, line);
            
            if (avgDist < minAvgDist) {
                minAvgDist = avgDist;
                minLine = line;
            }
        }
        
        assert (minLine == null);
        
        return new ArrayList<Point2D>(minLine);
    }
    
    /**
      * Determine the orientation of the line according to the direction of a 
      * set of GPS points. 
      */
    public List<Point2D> findCorrectOrientation(List<Point2D> line, List<GPSPoint> tempPoints) {
        Point2D from = line.get(0);
        Point2D to   = line.get(1);

        LonLat A = new LonLat(from.x(), from.y());
        LonLat B = new LonLat(to.x(), to.y());
        
        LonLat start = tempPoints.get(0).getLonLat();
        LonLat projStart = GeoCalculator.pointProjectionOnLineIgnoreInsideOrNot(start, A, B);
        
        LonLat end = tempPoints.get(tempPoints.size()-1).getLonLat();
        LonLat projEnd = GeoCalculator.pointProjectionOnLineIgnoreInsideOrNot(end, A, B);
        
        // projections of first point and last point
        double angle1 = GeoCalculator.calcAngle(projStart, projEnd);
        double angle2 = GeoCalculator.calcAngle(A, B);
        
        List<Point2D> newLine = new ArrayList<Point2D>(line);
        
        if (GeoCalculator.diffAngle180(angle1, angle2) > 1) {
            Collections.reverse(newLine);
        }
        
        return newLine;
    }
    
    /**
      * Calculate the approximation line for a minimum bounding box. 
      */
    public List<Point2D> calcApproxLine(List<GPSPoint> tempPoints, Boundary minBBox) {
        List<Point2D> approxLineSeg = approxDirectedLineSegment(tempPoints, minBBox);
        approxLineSeg = findCorrectOrientation(approxLineSeg, tempPoints);
        
        return approxLineSeg;
    }
    
    /**
      * Remove duplicates in a cluster. 
      */
    public List<Candidate> returnUniqueCandidates(List<GPSPoint> cluster) {
        Set<Candidate> hs = new HashSet<Candidate>();
        
        for (GPSPoint point : cluster) {
            hs.addAll(point.getCandidates());
        }
		
        List<Candidate> candidates = new ArrayList<Candidate>(hs);
        
        return candidates;
    }
    
    /**
      * Rotate a point based on a center and an angle. 
      */
    public LonLat rotateAPoint(LonLat p1, LonLat center, double angle) {
        //TRANSLATE TO ORIGIN
        double x1 = p1.getLon() - center.getLon();
        double y1 = p1.getLat() - center.getLat();
        
        //APPLY ROTATION
        double temp_x1 = x1 * Math.cos(angle) - y1 * Math.sin(angle);
        double temp_y1 = x1 * Math.sin(angle) + y1 * Math.cos(angle);
        
        return new LonLat(temp_x1 + center.getLon(), temp_y1 + center.getLat());
    }
    
    /**
      * Calculate the orientated rectangle. 
      */
    public List<LonLat> calcOrientedRect(LonLat from, LonLat to) {
        
        LonLat center = new LonLat(
            (from.getLon()+to.getLon())/2,
            (from.getLat()+to.getLat())/2
        );        

        double dist = GeoCalculator.euclideanDistance(from, to);
        double half = dist/2;
        double extended = Constant.OBR_EXTENDED_WIDTH;
        double angle = GeoCalculator.calcAngle(from, to);
                
        List<LonLat> oldRect = new ArrayList<LonLat>();
        oldRect.add(new LonLat(-(extended+half), -extended));
        oldRect.add(new LonLat(+(extended+half), -extended));
        oldRect.add(new LonLat(+(extended+half), +extended));
        oldRect.add(new LonLat(-(extended+half), +extended));

        List<LonLat> newRect = new ArrayList<LonLat>();
        LonLat origin = new LonLat(0,0);
        for (LonLat p : oldRect) {
            LonLat newP = rotateAPoint(p, origin, Math.toRadians(angle));
            newRect.add(new LonLat(newP.getLon()+center.getLon(), newP.getLat()+center.getLat()));
        }
        
        return newRect;
    }
    
    /**
      * Extract next 'k' points in a trajectory. 
      */
    List<GPSPoint> nextKPoints(int pos, int k, List<GPSPoint> traj) {
        if (pos + k > traj.size()-1) {
            return traj.subList(pos, traj.size());
        } else {
            return traj.subList(pos, pos+k+1);
        }
    }
    
    /**
      * Cluster GPS points in a trajectory. 
      */
    List<List<GPSPoint>> buildClusters(List<GPSPoint> traj, boolean isRemoved) {
        
        List<List<GPSPoint>> clusters = new ArrayList<List<GPSPoint>>();
        
        List<GPSPoint> currentWindow = new ArrayList<GPSPoint>();
        currentWindow.add(traj.get(0));
        currentWindow.add(traj.get(1));
        
        for (int i = 1; i < traj.size(); i++) {
            
            GPSPoint startPoint = currentWindow.get(0);
            GPSPoint endPoint   = currentWindow.get(currentWindow.size()-1);
            GPSPoint newEndPoint   = traj.get(i);
            LonLat startLonLat  = startPoint.getLonLat();
            LonLat endLonLat = endPoint.getLonLat();
            LonLat newEndLonLat    = newEndPoint.getLonLat();
            
            // Check internal
            int unsupported = 0;
            List<Boolean> seq = new ArrayList<Boolean>();
            seq.add(false);
            for (GPSPoint internalPoint : currentWindow.subList(1, currentWindow.size())) {
                
                double dist = GeoCalculator.distancePointToLineOnEuclideanDist(
                    internalPoint.getLonLat(), startLonLat, newEndLonLat
                );
                
                seq.add(dist > Constant.OBR_EXTENDED_WIDTH);
            }
            seq.add(false);
            
            double rate = ((double)unsupported)/(currentWindow.size()-1);
            boolean isSplit = false; 
            
            for (int j = 1; j < seq.size()-1; j++) {
                if (isRemoved) {
                    if (seq.get(j) && (seq.get(j-1) || seq.get(j+1) )) {
                        isSplit = true;
                        break;
                    }
                } else {
                    if (seq.get(j)) {
                        isSplit = true;
                        break;
                    }
                }
            }
            
            if (isSplit) {                
                // Create a new window
                clusters.add(currentWindow);
                GPSPoint lastP = currentWindow.get(currentWindow.size()-1);
                currentWindow = new ArrayList<GPSPoint>();
                currentWindow.add(lastP);
                currentWindow.add(newEndPoint);
            }
            else {
                currentWindow.add(traj.get(i));
            }
        }
        
        // Add the last window
        clusters.add(currentWindow);
        
        return clusters;
    }
    
    /**
      * Build axes from the clustering results. 
      */
    List<Point2D> buildAxes(List<List<GPSPoint>> clusters) {
        List<Point2D> axes = new ArrayList<Point2D>();
        
        for (int i = 0; i < clusters.size(); i++) {
            List<GPSPoint> cluster = clusters.get(i);
            LonLat from = cluster.get(0).getLonLat();
            LonLat to   = cluster.get(cluster.size()-1).getLonLat();
            
            axes.add(new Point2D(from.getLon(), from.getLat()));
            axes.add(new Point2D(to.getLon(),   to.getLat()));
        }
        
        return axes;
    }
    
    /**
      * Build rectangles from the clustering results. 
      */
    List<List<LonLat>> buildRectangles(List<List<GPSPoint>> clusters) {
        List<List<LonLat>> orientedRectangles = new ArrayList<List<LonLat>>();
        for (int i = 0; i < clusters.size(); i++) {
            List<GPSPoint> cluster = clusters.get(i);

            LonLat from = cluster.get(0).getLonLat();
            LonLat to   = cluster.get(cluster.size()-1).getLonLat();
            
            List<LonLat> orientedRect = calcOrientedRect(from, to);
            
            orientedRectangles.add(orientedRect);
        }
        
        return orientedRectangles;
    }
    
    /**
      * Build candidates for clusters. 
      */
    List<List<Candidate>> buildClusterCandidates(List<List<GPSPoint>> clusters) {
        List<List<Candidate>> clusterCandidates = new ArrayList<List<Candidate>>();
        for (int i = 0; i < clusters.size(); i++) {
            List<GPSPoint> cluster = clusters.get(i);
            
            clusterCandidates.add(returnUniqueCandidates(cluster));
        }
        
        return clusterCandidates;
    }
    
    /**
      * Rank edge candidates. 
      */
    List<Pair<Integer, Double>> rankingStreets(double axisAngle, List<Candidate> candidates, 
        Point2D point, boolean useAxisAngle) {
        
        Map<Integer, Double> mapStreetToWeight = new HashMap<Integer, Double>();
                
        for (Candidate cand: candidates) {
            
            DirectedEdge edge = this.G.edge(cand.getStreetID());
            LonLat edgeStart = this.nodes.get(edge.from()).getLonLat();
            LonLat edgeEnd   = this.nodes.get(edge.to()).getLonLat();
            
            double candAngle = GeoCalculator.calcAngle(edgeStart,edgeEnd);
            double angleDiff = GeoCalculator.diffAngle180(candAngle,axisAngle);

            double dist = GeoCalculator.distancePointToProjectedLine(
                new LonLat(point.x(), point.y()), edgeStart, edgeEnd);
            
            // If the distance is greater than the defined maximum distance
            if (dist > Constant.MAX_DIST_METERS) continue;
            
            double angleWeight = 1 - angleDiff/180;
            
            double distWeight = this.norm.compute(dist);
            
            double totalWeight; 
            
            if (useAxisAngle) {
                totalWeight = angleWeight*distWeight;
            }
            else {
                // only use distance weight
                totalWeight = distWeight;
            }
            
            mapStreetToWeight.put(cand.getStreetID(), totalWeight);
        }
        
        List<Pair<Integer, Double>> pairs = new ArrayList<Pair<Integer, Double>>();
        
        for (Map.Entry<Integer, Double> entry: mapStreetToWeight.entrySet()) {
            // Remove some low probable choices
            if (entry.getValue() < Constant.EPSILON) continue;
            
            pairs.add(new Pair<Integer, Double>(entry.getKey(), entry.getValue()));
        }
        
        // Sort the result
        Collections.sort(pairs, new Comparator<Pair<Integer, Double>>() {
            @Override
            public int compare(Pair<Integer, Double> o1, Pair<Integer, Double> o2) {
                if      (o1.y > o2.y)        return -1;
                else if (o1.y < o2.y)        return 1;
                else                         return 0;
            }
        });
        
        return pairs;
    }
    
    /**
      * Check whether or not a point is inside a rectangle. 
      */
    boolean isInRect(LonLat point, List<LonLat> rect) {
        LonLat A = rect.get(0);
        LonLat B = rect.get(1);
        LonLat C = rect.get(2);
        
        LonLat proj1 = GeoCalculator.pointProjectionOnLineIgnoreInsideOrNot(
            point, A, B
        );
        
        LonLat proj2 = GeoCalculator.pointProjectionOnLineIgnoreInsideOrNot(
            point, B, C
        );
        
        return GeoCalculator.isInLineBounds(proj1, A, B) 
            && GeoCalculator.isInLineBounds(proj2, B, C);
    }
    
    /**
      * Find all possible travel paths for a cluster. 
      */
    List<List<Integer>> findAllPossiblePaths(int fromStreetID, List<LonLat> rect, Set<Integer> terminalEdges, 
        double axisAngle) {
        
        List<List<Integer>> paths = new ArrayList<List<Integer>>();
        DirectedEdge fromStreet = this.G.edge(fromStreetID);
        OSMNode fromNode = this.nodes.get(fromStreet.from());
        OSMNode toNode = this.nodes.get(fromStreet.to());
        
        if (!isInRect(toNode.getLonLat(), rect)) {
            List<Integer> path = new ArrayList<Integer>();
            path.add(fromStreetID);
            paths.add(path);
            return paths;
        }
        
        Set<Integer> visitedNodes = new HashSet<Integer>();
        visitedNodes.add(fromNode.getIndex());
        visitedNodes.add(toNode.getIndex());
        
        Map<Integer, Pair<Double, Integer>> trackbackMap = new HashMap<Integer, Pair<Double, Integer>>();
        
        double edgeAngle = GeoCalculator.calcAngle(fromNode.getLonLat(), toNode.getLonLat());
        double edgeAngleDiff = GeoCalculator.diffAngle180(axisAngle, edgeAngle);
        trackbackMap.put(toNode.getIndex(), new Pair<Double, Integer>(edgeAngleDiff, fromStreetID));
        
        List<Integer> endEdges = new ArrayList<Integer>();
        
        int currentNode;
        Queue<Integer> queue = new LinkedList<Integer>();
        queue.add(toNode.getIndex());
                
        while (!queue.isEmpty()) {
            currentNode = queue.poll();
            
            double prevDiff = trackbackMap.get(currentNode).x;
            for (DirectedEdge edge : this.G.adj(currentNode)) {
                int prevNodeID = edge.from();
                int nextNodeID = edge.to();
                fromNode = this.nodes.get(prevNodeID);
                toNode   = this.nodes.get(nextNodeID);
                edgeAngle = GeoCalculator.calcAngle(fromNode.getLonLat(), toNode.getLonLat());
                edgeAngleDiff = GeoCalculator.diffAngle180(axisAngle, edgeAngle);
                
                if (trackbackMap.get(nextNodeID) != null && (prevDiff + edgeAngleDiff > trackbackMap.get(nextNodeID).x)) {
                    continue;
                }
                
                trackbackMap.put(
                    nextNodeID, 
                    new Pair<Double, Integer>(prevDiff + edgeAngleDiff, edge.id())
                );
                
                LonLat nextLonLat = toNode.getLonLat();
                if (terminalEdges.contains(edge.id())) {
                    endEdges.add(edge.id());
                }
                else if (isInRect(nextLonLat, rect) && !visitedNodes.contains(nextNodeID)) {
                    visitedNodes.add(nextNodeID);
                    queue.offer(nextNodeID);
                }
            }
        }
        
        // Search all possible paths
        for (Integer endEdge : endEdges) {
            List<Integer> path = new ArrayList<Integer>();
            
            int endNodeID = this.G.edge(endEdge).to();
            int currentStreetID  = trackbackMap.get(endNodeID).y;
            while (currentStreetID != fromStreetID) {
                path.add(currentStreetID);
                currentStreetID = trackbackMap.get(this.G.edge(currentStreetID).from()).y;
            }
            path.add(fromStreetID);
            
            Collections.reverse(path);
            
            paths.add(path);
        }
        
        if (paths.size() == 0) {
            List<Integer> path = new ArrayList<Integer>();
            path.add(fromStreetID);
            paths.add(path);
        }
        
        return paths;
    }
    
    /**
      * Build a simple route candidate graph. 
      */
    List<List<Pair<List<Integer>,Double>>> buildSimpleRouteCandidateGraph(List<List<GPSPoint>> clusters,
        List<Point2D> axes, List<List<Candidate>> clusterCandidates) {
        List<List<Pair<List<Integer>,Double>>> possiblePathsForAllClusters = 
            new ArrayList<List<Pair<List<Integer>,Double>>>();

        System.out.println("original clusters' size:" + clusters.size());

        for (int i = 0; i < clusters.size(); i++) {
            // Get the axis 
            Point2D axisStart = axes.get(i*2);
            Point2D axisEnd   = axes.get(i*2+1);
            double axisAngle = GeoCalculator.calcAngle(axisStart, axisEnd);
            
            // Rank the first streets in the cluster (proximity)
            List<Pair<Integer, Double>> firstStreetsOfCluster = rankingStreets(
                axisAngle,
                clusterCandidates.get(i),
                axes.get(i*2), // The start point of axis
                false
            );

            List<Pair<List<Integer>,Double>> possiblePathPairs = 
                new ArrayList<Pair<List<Integer>,Double>>();
            
            // The route only contains the first street without internal sub-routes
            for (Pair<Integer, Double> pair : firstStreetsOfCluster) {
                List<Integer> route = new ArrayList<Integer>();
                route.add(pair.x);
                possiblePathPairs.add(new Pair<List<Integer>,Double>(route, pair.y));
            }

            possiblePathsForAllClusters.add(possiblePathPairs);

            if (i == clusters.size()-1) {
                List<Pair<Integer, Double>> lastStreetsOfCluster = rankingStreets(
                    axisAngle,
                    clusterCandidates.get(i),
                    axes.get(i*2+1), // The end point of axis
                    false
                );
                
                possiblePathPairs = new ArrayList<Pair<List<Integer>,Double>>();

                for (Pair<Integer, Double> pair : lastStreetsOfCluster) {
                    List<Integer> route = new ArrayList<Integer>();
                    route.add(pair.x);
                    possiblePathPairs.add(new Pair<List<Integer>,Double>(route, pair.y));
                }

                possiblePathsForAllClusters.add(possiblePathPairs);
            }
        }

        return possiblePathsForAllClusters;
    }
    
    /**
      * Build a route candidate graph. 
      */
    List<List<Pair<List<Integer>,Double>>> buildRouteCandidateGraph(List<List<GPSPoint>> clusters, List<Point2D> axes, 
        List<List<LonLat>> orientedRectangles, List<List<Candidate>> clusterCandidates) {
        
        List<List<Pair<List<Integer>,Double>>> possiblePathsForAllClusters = 
            new ArrayList<List<Pair<List<Integer>,Double>>>();
        
        for (int i = 0; i < clusters.size(); i++) {
            Point2D axisStart = axes.get(i*2);
            Point2D axisEnd   = axes.get(i*2+1);
            LonLat lonlatAxisStart = new LonLat(axisStart.x(), axisStart.y());
            LonLat lonlatAxisEnd   = new LonLat(axisEnd.x(), axisEnd.y());
            double axisDist = GeoCalculator.euclideanDistance(lonlatAxisStart, lonlatAxisEnd);
            double axisAngle = GeoCalculator.calcAngle(axisStart, axisEnd);

            List<Pair<Integer, Double>> firstStreetsOfCluster = rankingStreets(
                axisAngle,
                clusterCandidates.get(i),
                axes.get(i*2), 
                true
            );
                        
            List<Pair<Integer, Double>> lastStreetsOfCluster = rankingStreets(
                axisAngle,
                clusterCandidates.get(i),
                axes.get(i*2+1),
                true
            );
                        
            if (firstStreetsOfCluster.size() == 0) {
                System.out.println("NO first cluster!!!");
                possiblePathsForAllClusters.add(new ArrayList<Pair<List<Integer>,Double>>());
                continue;
            }
            
            List<List<Integer>> possiblePaths = new ArrayList<List<Integer>>();
            
            // Add all terminal edges into a list
            Set<Integer> terminalEdges = new HashSet<Integer>();
            for (Pair<Integer, Double> pair : lastStreetsOfCluster) {
                terminalEdges.add(pair.x);
            }
            
            for (Pair<Integer, Double> pair : firstStreetsOfCluster) {
                // Find all possible paths starting from first streets
                possiblePaths.addAll(findAllPossiblePaths(
                    pair.x, orientedRectangles.get(i), terminalEdges, axisAngle
                ));
            }
            
            List<Pair<List<Integer>,Double>> possiblePathPairs = new ArrayList<Pair<List<Integer>,Double>>();
            
            for (List<Integer> path : possiblePaths) {
                // For all possible paths
                double totalAngleWeight = 0;
                double lengthOfPath = 0;
                
                for (Integer streetID : path) {
                    DirectedEdge edge = this.G.edge(streetID);
                    
                    lengthOfPath += edge.weight();
                    
                    LonLat edgeStart = this.nodes.get(edge.from()).getLonLat();
                    LonLat edgeEnd   = this.nodes.get(edge.to()).getLonLat();
                    
                    double candAngle = GeoCalculator.calcAngle(edgeStart,edgeEnd);
                    double angleDiff = GeoCalculator.diffAngle180(candAngle,axisAngle);
                    
                    double angleWeight = 1 - angleDiff/180;
                    
                    // Angle weight
                    totalAngleWeight += angleWeight;
                }
                
                totalAngleWeight  = totalAngleWeight / path.size();
                
                possiblePathPairs.add(new Pair<List<Integer>,Double>(path, totalAngleWeight));
            }
            
            if (possiblePathPairs.size() == 0) {
                // Add an empty list
                System.out.println("No possible paths:" + i);
                possiblePathsForAllClusters.add(new ArrayList<Pair<List<Integer>,Double>>());
                System.out.println("size:" + possiblePathsForAllClusters.size());
                continue;
            }
            
            // Sort the result
            Collections.sort(possiblePathPairs, new Comparator<Pair<List<Integer>, Double>>() {
                @Override
                public int compare(Pair<List<Integer>, Double> o1, Pair<List<Integer>, Double> o2) {
                    if      (o1.y > o2.y)        return -1;
                    else if (o1.y < o2.y)        return 1;
                    else                         return 0;
                }
            });
            
            possiblePathsForAllClusters.add(possiblePathPairs);
        }
        
        return possiblePathsForAllClusters;
    }
    
    /**
      * Calculate the distance between two points. 
      */
    double calcRouteDist(int startNodeID, int endNodeID) {
                
        sp.searchSP(startNodeID, endNodeID);
        
        if (!sp.hasPathTo(endNodeID)) {
            return Constant.IMPOSSIBLE_DIST;
        }
        
        return sp.distTo(endNodeID);
    }
    
    
    /**
      * Compute the transition probability. 
      */
    double compTransProb(List<Integer> prevPath, List<Integer> currPath, Map<String, Double> routesMap) {
        
        Set<Integer> prevSet = new HashSet<Integer>(prevPath);
        
        if (prevSet.contains(currPath.get(0))) {
            return 1;
        }
        
        int startNodeID = this.G.edge(prevPath.get(prevPath.size()-1)).to();
        int endNodeID   = this.G.edge(currPath.get(0)).from();
        
        if (startNodeID == endNodeID) {
            return 1;
        }
        
        LonLat startNode = this.nodes.get(startNodeID).getLonLat();
        LonLat endNode   = this.nodes.get(endNodeID).getLonLat();
        
        // Compute the direct distance between two points.
        double greatCircleDist = GeoCalculator.calculateDistance(startNode, endNode);
                
        String key = String.format("%d %d", startNodeID, endNodeID);
        
        // Compute the route distance between two points. If it has been 
        // calculated before, then just retrieve it. Otherwise, recalculate
        // it and store it into the table of routes. 
        double routeDist = 0;
        if (routesMap.containsKey(key)) {
            routeDist = routesMap.get(key);
        } else {
            routeDist = calcRouteDist(startNodeID, endNodeID);
            routesMap.put(key, routeDist);
        }
        
		return this.exp.compute(Math.abs(routeDist-greatCircleDist));
    }
    
    /**
      * Estimate the footprint. 
      */
    List<Integer> buildFootprint(List<List<GPSPoint>> clusters, List<Point2D> axes, 
        List<List<LonLat>> orientedRectangles, List<List<Candidate>> clusterCandidates) {
        
        List<Integer> footprint = new ArrayList<Integer>();
        
        List<List<Pair<List<Integer>,Double>>> possiblePathsForAllClusters;
        
        // System.out.println("### Build Route Graph...");
        if (this.useRect) {
            // System.out.println("#### Use The Rectangle Strategy...");
            possiblePathsForAllClusters = buildRouteCandidateGraph(clusters, axes, orientedRectangles, clusterCandidates);
        }
        else
        {
            // System.out.println("#### Only Use Anchor Points...");
            possiblePathsForAllClusters = buildSimpleRouteCandidateGraph(clusters, axes, clusterCandidates);
        }
        
        // System.out.println("### Build Route Network...");
        List<List<Integer>> allRoutes = new ArrayList<List<Integer>>();
        List<Pair<Integer,List<Node>>> network = new ArrayList<Pair<Integer,List<Node>>>();
        
        for (int i = 0; i < possiblePathsForAllClusters.size(); i++) {
            
            List<Pair<List<Integer>,Double>> possiblePathsPairs = possiblePathsForAllClusters.get(i);
            
            if (possiblePathsPairs.size() == 0) continue;
            
            List<Node> nodesForSingleCluster = new ArrayList<Node>();
            
            for (Pair<List<Integer>, Double> pathPair : possiblePathsPairs) {
                
                nodesForSingleCluster.add(new Node(allRoutes.size(), pathPair.y));
                allRoutes.add(pathPair.x);
            }
            
            network.add(new Pair<Integer,List<Node>>(i, nodesForSingleCluster));
        }
        
        
        // System.out.println("### Calculate Probabilities...");
        // Calculate the transition probability
        Map<String, Double> routesMap = new HashMap<String, Double>();
        // The first cluster has no prior nodes, so that their total 
        // probability equals to their emission probability. 
        for (Node node : network.get(0).y) {
            node.setProb(Math.log(node.emission()));
        }
        
        for (int i = 1; i < network.size(); i++) {
            List<Node> prevNodes = network.get(i-1).y;
            List<Node> currNodes = network.get(i).y;
            
            for (Node curr : currNodes) {
                
                double emissProb = curr.emission();
                
                for (Node prev : prevNodes) {
                    List<Integer> prevPath = allRoutes.get(prev.routeID());
                    List<Integer> currPath = allRoutes.get(curr.routeID());
                                        
                    double transProb = compTransProb(prevPath, currPath, routesMap);
                    
                    double currProb = emissProb * transProb;
                    
                    double possibleProb = Math.log(currProb) + prev.prob();
                    
                    if (possibleProb > curr.prob()) {
						curr.setProb(possibleProb);
						curr.setPrev(prev);
                    }
                }
                
            }
        }
        
        // System.out.println("### Search Best Route...");
        List<Node> nodesLastCluster = network.get(network.size()-1).y;
        Node maxNode = nodesLastCluster.get(0);
        double maxValue = maxNode.prob();
        for (int i = 1; i < nodesLastCluster.size(); i++) {
            if (nodesLastCluster.get(i).prob() > maxValue) {
                maxNode  = nodesLastCluster.get(i);
                maxValue = nodesLastCluster.get(i).prob();
            }
        }
        
        Node ptr = maxNode;
        List<Integer> optimalRoutePath = new ArrayList<Integer>();
        
        while (ptr != null) {
            optimalRoutePath.add(ptr.routeID());
            ptr = ptr.prev();
        }
        
        Collections.reverse(optimalRoutePath);
        
        List<List<Integer>> solveOverlapSolution = new ArrayList<List<Integer>>();
        solveOverlapSolution.add(allRoutes.get(optimalRoutePath.get(0)));
        
        for (int i = 1; i < optimalRoutePath.size(); i++) {
            List<Integer> prevRoute = allRoutes.get(optimalRoutePath.get(i-1));
            List<Integer> currRoute = allRoutes.get(optimalRoutePath.get(i));

            Set<Integer> prevSet = new HashSet<Integer>(prevRoute);

            if (!prevSet.contains(currRoute.get(0))) {
                solveOverlapSolution.add(currRoute);
                continue;
            }
            
            int repeatedIndex = currRoute.indexOf(prevRoute.get(prevRoute.size()-1));
            List<Integer> prunedRoute = currRoute.subList(repeatedIndex+1, currRoute.size());
            if (prunedRoute.size() != 0) {
                solveOverlapSolution.add(prunedRoute);
            }
        }
        
        // System.out.println("### Conclude the footprint...");
        footprint.addAll(solveOverlapSolution.get(0));
        for (int i = 1; i < solveOverlapSolution.size(); i++) {
            List<Integer> prevRoute = solveOverlapSolution.get(i-1);
            List<Integer> currRoute = solveOverlapSolution.get(i);
            
            int startNodeID = this.G.edge(prevRoute.get(prevRoute.size()-1)).to();
            int endNodeID   = this.G.edge(currRoute.get(0)).from();

            sp.searchSP(startNodeID, endNodeID);

            if (!sp.hasPathTo(endNodeID)) {
                System.out.println("No Path!!!");
                continue;
            }

            for (DirectedEdge edge : sp.pathTo(endNodeID)) {
                footprint.add(edge.id());
            }

            footprint.addAll(currRoute);
        }

        return footprint;
    }
    
    /**
      * Write out the approximation lines. 
      */
    <T> void outputApproxLineSegs(List<T> approxLineSegs) throws IOException {
        
        File outputFile = new File(outFilename + "_approx.txt");
        
        if (!outputFile.exists()) {
            outputFile.createNewFile();
        }
        
        FileWriter fw = new FileWriter(outputFile.getAbsoluteFile());
        BufferedWriter bw = new BufferedWriter(fw);
        
        bw.write(approxLineSegs.size() + "\n");
        
        for (T p: approxLineSegs) {
            bw.write(p + "\n");
        }
        
        bw.close();
        fw.close();
    }
    
    /**
      * Write out the oriented rectangles. 
      */
    void outputOrientedRectangles(List<List<LonLat>> orientedRects) throws IOException {
        File outputFile = new File(outFilename + "_oriented.txt");
        
        if (!outputFile.exists()) {
            outputFile.createNewFile();
        }
        
        FileWriter fw = new FileWriter(outputFile.getAbsoluteFile());
        BufferedWriter bw = new BufferedWriter(fw);
        
        bw.write(orientedRects.size() + "\n");
        
        for (List<LonLat> rect: orientedRects) {
            for (LonLat p: rect) {
                bw.write(String.format("%.6f %.6f ", p.getLon(), p.getLat()));
            }
            bw.write("\n");
        }
        
        bw.close();
        fw.close();
    }
    
    /**
      * The entry of this instance to infer footprint for a trajectory. 
      */
    public List<Integer> computeFootprint(List<GPSPoint> traj) {
        
        // Compute the density of trajectory, decide whether the 
        // obr algorithm should throw out points or not. 
        double densityTraj = computeDensity(traj);
        boolean isRemoved = densityTraj > Constant.OBR_TRAJ_SPARSE_DENSITY;
        
        // Build spatial-linear clusters
        List<List<GPSPoint>> clusters = buildClusters(traj, isRemoved);
        
        // Build axes & rectangles
        List<Point2D> axes = buildAxes(clusters);
        List<List<LonLat>> orientedRectangles = buildRectangles(clusters);
        
        // Build candidates for clusters
        List<List<Candidate>> clusterCandidates = buildClusterCandidates(clusters);
        
        // search k paths for each cluster, pruning route tree
        List<Integer> footprint = buildFootprint(clusters, axes, orientedRectangles, 
            clusterCandidates);
        
        // Output the approximation lines and oriented rectangles for visualization. 
        // try {
        //     outputApproxLineSegs(axes);
        //     outputOrientedRectangles(orientedRectangles);
        // } catch (Exception ex) {
        //     ex.printStackTrace();
        // }
        
        return footprint;
    }
    
    // Some private helper classes are defined as follows: 
    private class Pair<X, Y> { 
        public final X x; 
        public final Y y;
        public Pair(X x, Y y) { 
            this.x = x;
            this.y = y;
        }
    }     
    
    private class Tuple<X, Y, Z> { 
        public final X x; 
        public final Y y;
        public final Z z;
        public Tuple(X x, Y y, Z z) { 
            this.x = x;
            this.y = y;
            this.z = z;
        }
    }
    
    private class Node {
        Node prev;
        final int routeID;
        final double emissionProb;
        double prob;
        
        public Node(int routeID, double emissionProb) {
            this.routeID = routeID;
            this.emissionProb = emissionProb;
            this.prev = null;
            this.prob = -Double.MAX_VALUE;
        }
        
        public int routeID() { return this.routeID; }
        public double emission() { return this.emissionProb; }
        public Node prev() { return this.prev; }
        public void setPrev(Node prev) { this.prev = prev; }
        public double prob() { return this.prob; }
        public void setProb(double prob) { this.prob = prob; }
    }
}
