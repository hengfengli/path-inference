package pathinfer.main;

import java.io.*;
import java.util.List;
import java.util.Set;
import java.util.HashSet;
import java.util.Scanner;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

import edu.princeton.cs.algs4.Stack;
import edu.princeton.cs.algs4.Stopwatch;

import pathinfer.map.*;
import pathinfer.geo.*;
import pathinfer.graph.*;
import pathinfer.simp.*;
import pathinfer.obr.*;

import pathinfer.stats.NormalDistribution;
import pathinfer.stats.ExponentialDistribution;

import pathinfer.main.Constant;


/**
 *  This class provides the major entry of path inference program.  
 *  
 *  @author Hengfeng Li
 */
public class PathInference {
	    
    private EdgeWeightedDigraph G;
    private DijkstraSP sp;
    private List<OSMNode> nodes;
    private List<Street> streets;
	
	private final ExponentialDistribution exp;
	private final NormalDistribution norm;
	
    private GridPartition gridPartition; 
	// Constant comparator
	private final CandidateComparable candidateComparator = new CandidateComparable();
	
    /**
      * Constructor. 
      */
	public PathInference(EdgeWeightedDigraph G, DijkstraSP sp, List<OSMNode> 
        nodes, List<Street> streets, GridPartition gridPartition) {
		this.gridPartition = gridPartition;
        
        this.G     = G;
        this.sp    = sp;
        this.nodes = nodes;
        this.streets = streets;
		
        this.norm = new NormalDistribution(0, Constant.NOISE_SIGMA);
		this.exp = new ExponentialDistribution(Constant.EXP_ALPHA, Constant.EXP_BETA);
	}
	
    /**
      * Search possible candidates for a GPS point. 
      */
	public void searchCandidatesForOnePoint(GPSPoint p, List<Street> searchGrid) {
        
		// Initilize
		List<Candidate> candidates = new ArrayList<Candidate>();
		LonLat point = p.getLonLat();
        		
		for (int i = 0; i < searchGrid.size(); i++) {
			// All these computation are based on Euclidean Distance. 
			Street close_street = searchGrid.get(i);
			LonLat startnode_lonlat = close_street.getStartNode().getLonLat();
			LonLat endnode_lonlat   = close_street.getEndNode().getLonLat();
		           	
			LonLat new_lonlat = GeoCalculator.pointProjectionOnLine(
			    point,
		        startnode_lonlat,
		        endnode_lonlat);
		
			double distance = GeoCalculator.calculateDistance(point, 
				new_lonlat);
			
            if (distance < Constant.MAX_DIST_METERS) {
                candidates.add(new Candidate(close_street.getStreetID(), distance));
            }
		}
        
		// remove duplicates
		Set<Candidate> hs = new HashSet<Candidate>();
		hs.addAll(candidates);
		candidates.clear();
		candidates.addAll(hs);
		
		// sort the candidates according to its distance to
		// the point in an ascending order.
		Collections.sort(candidates, candidateComparator);
		
		if (candidates.size() <= Constant.NUM_CANDIDATES) {
			p.setCandidates(candidates);
		} else {
			p.setCandidates(candidates.subList(0, Constant.NUM_CANDIDATES));
		}
	}
	
    /**
      * Search possible candidates for all points in a trajectory. 
      */
	public void findCandidatesForOneTraj(List<GPSPoint> traj, List<List<Street>> gridIndex) {
		
		int x, y;
		double centerX, centerY;
					
		for (GPSPoint gpspoint : traj) {
			
			centerX = gpspoint.getLon();
			centerY = gpspoint.getLat();
			
			// search the grids this gps point's rectangle covers
			List<Integer> coveredGridIndexes = 
				this.gridPartition.findCoveringGrids(
                                  new LonLat(centerX-Constant.RECT_SIZE, 
											 centerY-Constant.RECT_SIZE), 
								  new LonLat(centerX+Constant.RECT_SIZE,
											 centerY+Constant.RECT_SIZE));
			
			List<Street> searchGrids = new ArrayList<Street>();
			
			// aggregate all streets that need to be searched
			for (Integer index : coveredGridIndexes) {
				searchGrids.addAll(gridIndex.get(index));
			}
			
			// search the nearest candidates
			searchCandidatesForOnePoint(gpspoint, searchGrids);
		}
	}
    
    
	/**
	 * Compute distance from the projected point to start node
	 * or end node. 
	 */
	public static double computeProjectedDistanceOnEdge(LonLat point, 
		LonLat start, LonLat end, boolean isToEnd) {
		// compute the projection point of previous GPS point
		LonLat projectedPoint = GeoCalculator.pointProjectionOnLine(
			point,
			start,
			end
		);
		
		LonLat target = isToEnd ? end : start;
		
		return GeoCalculator.calculateDistance(
			projectedPoint,
			target
		);
	}
	
	/**
	 * Compute the transition probability. 
	 */
	public double computeTransitionProb(Candidate prev, Candidate curr,
		GPSPoint prevPoint, GPSPoint currPoint) {
		
		double pointsDistance = GeoCalculator.calculateDistance(
			prevPoint.getLonLat(), 
			currPoint.getLonLat()
		);
		
		DirectedEdge e1 = G.edge(prev.getStreetID());
		DirectedEdge e2 = G.edge(curr.getStreetID());
		
		if (e1.id() == e2.id()) {
			double prevDist = computeProjectedDistanceOnEdge(
				prevPoint.getLonLat(),
				nodes.get(e1.from()).getLonLat(),
				nodes.get(e1.to()).getLonLat(),
				true
			);
			
			double currDist = computeProjectedDistanceOnEdge(
				currPoint.getLonLat(),
				nodes.get(e1.from()).getLonLat(),
				nodes.get(e1.to()).getLonLat(),
				true
			);
			
			return exp.compute(
				Math.abs((currDist - prevDist) - pointsDistance));
		}
		
		// if two candidates are located in different edges,
		// then we need to compute the distance of route between
		// them.  
		
		// compute the distance from the projected point to the 
		// end node of the edge.
		double startDistance = computeProjectedDistanceOnEdge(
			prevPoint.getLonLat(),
			nodes.get(e1.from()).getLonLat(),
			nodes.get(e1.to()).getLonLat(),
			true
		);
		// compute the distance from the start node of the edge 
		// to the projected point.
		double endDistance = computeProjectedDistanceOnEdge(
			currPoint.getLonLat(),
			nodes.get(e2.from()).getLonLat(),
			nodes.get(e2.to()).getLonLat(),
			false
		);
		
		sp.searchSP(e1.to(), e2.from());
		
		// if there is no shortest path between them, 
		// then return a very small probability.
		if (!sp.hasPathTo(e2.from())) {
			return this.exp.compute(Constant.IMPOSSIBLE_DIST);
		}
				
		return this.exp.compute(
			Math.abs(
				(startDistance + sp.distTo(e2.from()) + endDistance)
			  - pointsDistance)
			);
	}
	
	/**
	 * Compute the emission probability. 
	 */
	public double computeEmissionProb(double dist) {
		return this.norm.compute(dist);
	}
	
	/**
	 * Conduct the forward propagation to calculate probability for every node 
     * in the network. 
	 */
	public void vertbi(List<GPSPoint> traj) {
        
		// Compute initial probability with only considering the candidate's 
		// distance to GPS point
		for (Candidate candiate: traj.get(0).getCandidates()) {
			double emissionProb = computeEmissionProb(candiate.getDistance());
			candiate.setProb(Math.log(emissionProb));
		}
		
		for (int i = 1; i < traj.size(); i++) {
			GPSPoint prevPoint = traj.get(i-1);
			GPSPoint currPoint = traj.get(i);
						
			// current candidates choose a best previous route
			for (Candidate curr: currPoint.getCandidates()) {
				
				// emission prob
				double emissionProb = computeEmissionProb(curr.getDistance());
				
				for (Candidate prev: prevPoint.getCandidates()) {
					
					// transition prob
					double transitionProb = computeTransitionProb(prev, curr, 
						prevPoint, currPoint);
					
					// previous route
					double prevProb = prev.prob();
										
                    if (transitionProb < Constant.EPSILON) {
                        transitionProb = Constant.EPSILON;
                    }
					
                    double currProb = emissionProb * transitionProb;
					
					// logarithm
					double possibleProb = Math.log(currProb) + prevProb;
										
					if (possibleProb > curr.prob()) {
						curr.setProb(possibleProb);
						curr.setPrev(prev);
					}
					
				}
				
			}
			
		}
	}
	
	/**
	 * A simplest strategy of path inference to look for the closest road 
     * segment. 
	 */
	public Iterable<Integer> cloestRoadSegment(List<GPSPoint> traj) {
        
		List<Integer> path = new ArrayList<Integer>();
		Candidate best;
		
		for (int i = 0; i < traj.size(); i++) {
			List<Candidate> candidates = traj.get(i).getCandidates();
			best = null;
            
            for (Candidate candidate : candidates) {
                if (best == null || candidate.getDistance() < best.getDistance()) {
                    best = candidate;
                }
            }
            
            if (best == null) {
                System.out.println("i:" + i);
                System.out.println("GPS point:" + traj.get(i));
                System.out.println("candidates' size:" + candidates.size());
                System.out.println("BEST IS NULL!");
                continue;
            }

            path.add(best.getStreetID());
		}
		
		return path;
	}
	
	/**
	 * The incremental algorithm from the paper "On map-matching vehicle 
     * tracking data". 
	 */
	public Iterable<Integer> incremental(List<GPSPoint> traj) {
        
		List<Integer> path = new ArrayList<Integer>();

		double bestScore = 0.0;
		double s_d = 0.0;
		double s_a = 0.0;
		double s = 0.0;
		
		// i == 0
		GPSPoint first = traj.get(0);
		List<Candidate> candidates = first.getCandidates();
		
        Candidate best = null;
		for (Candidate candidate : candidates) {
			if (best == null || candidate.getDistance() < best.getDistance()) {
				best = candidate;
			}
		}
		path.add(best.getStreetID());
		
		// i = 1, 2, ..., n
		for (int i = 1; i < traj.size(); i++) {
			GPSPoint prev = traj.get(i-1);
			GPSPoint curr = traj.get(i);
			Integer bestChoice = -1;
			
            // calculate the angle of trajectory segment
			double pointsAngle = GeoCalculator.calcAngle(prev.getLonLat(), curr.getLonLat());
            
            // get next choices (includes itself)
            List<Integer> nextChoices = new ArrayList<Integer>();
            
            // add current street ID
            nextChoices.add(path.get(path.size()-1));
            
            DirectedEdge currentEdge = G.edge(path.get(path.size()-1));
            
            for (DirectedEdge edge: G.adj(currentEdge.to())) {
                nextChoices.add(edge.id());
            }
            
            for (Integer choice: nextChoices) {
                DirectedEdge edge = G.edge(choice);
                LonLat start = nodes.get(edge.from()).getLonLat();
                LonLat end   = nodes.get(edge.to()).getLonLat();
                
    			LonLat projection = GeoCalculator.pointProjectionOnLine(
    			    curr.getLonLat(),
    		        start,
    		        end
                );
	
    			double distance = GeoCalculator.calculateDistance(
                    curr.getLonLat(),
    				projection
                );
                
                s_d = 10 - 0.17 * Math.pow(distance, 1.4);
                
                // calculate the angle of road segment
                double edgeAngle = GeoCalculator.calcAngle(start, end);
                
                s_a = 10 * Math.pow(
                    Math.cos(GeoCalculator.diffTurningAngle(pointsAngle, edgeAngle)), 
                    4
                );
                
                s = s_a + s_d;
                
                if (bestChoice == -1 || s > bestScore) {
                    bestScore = s;
                    bestChoice = choice;
                }
            }
            
            if (bestChoice == -1) {
                System.out.println("GPS point:" + traj.get(i));
                System.out.println("BEST IS NULL!");
                continue;
            }
            
            path.add(bestChoice);
		}
		
		return path;
	}
	
	/**
	 * Calculate the optimal path on the markov network (backward propagation)
     * for HMM. 
	 */
	public Iterable<Integer> hmm(List<GPSPoint> traj) {
        
		Stack<Integer> path = new Stack<Integer>();
		
		// Back-tracking and put the optimal path into 
		// a stack. 
		Candidate best = null;
		double maxProb = -Double.MAX_VALUE;
		
		List<Candidate> candidatesForLastPoint 
			= traj.get(traj.size()-1).getCandidates();
		
		if (candidatesForLastPoint == null) System.out.println("NULL!!!!");
		if (candidatesForLastPoint != null && candidatesForLastPoint.size() != 0) {
			// From the last point to backtrack
			for (Candidate candidate : candidatesForLastPoint) {
				if (candidate.prob() > maxProb) {
					maxProb = candidate.prob();
					best = candidate;
				}
			}
			
			if (best != null) {
				// Start from the best one
				path.push(best.getStreetID());
				for (Candidate c = best.prev(); c != null; c = c.prev()) {
					// Move to the previous one
					path.push(c.getStreetID());
				}
			}
		}
		
		return path;
	}
	
	/**
	 * Calculate the optimal path on the markov network (backward propagation). 
	 */
	public Iterable<Integer> computeOptimalPath(List<GPSPoint> traj, Constant.AlgoMode mode) {
		
		Iterable<Integer> path;
		
		if (mode == Constant.AlgoMode.HMM || mode == Constant.AlgoMode.FILTER) {
			path = hmm(traj);
		} else if (mode == Constant.AlgoMode.INCREMENT) {
			path = incremental(traj);
		} else {
			path = cloestRoadSegment(traj);
		}
		
		return path;
	}
    
    /**
     * This generic method convert an iterator to a list. 
     */
    private <T> List<T> convertIterableToList(Iterable<T> iter) {
        List<T> list = new ArrayList<T>();
        
        for (T item: iter) list.add(item);
        
        return list;
    }
    
	/**
	 * Write out the estimated path. 
	 */
    public void writeEstimatedPath(List<Integer> edgesID, String outFilePath) throws IOException{
        
		File outputFile = new File(outFilePath);
        
        if (!outputFile.exists()) {
            outputFile.createNewFile();
        }
        
        FileWriter fw = new FileWriter(outputFile.getAbsoluteFile());
        BufferedWriter bw = new BufferedWriter(fw);
        
        bw.write(edgesID.size() + "\n");
        
        for (Integer id : edgesID) {
            bw.write(id + "\n");
        }
        
        bw.close();
    }
    
	/**
	 * Remove duplicates in the sequence of edge IDs. 
	 */
    List<Integer> removeDuplicates(List<Integer> edgesID) {
        int current = edgesID.get(0);
        List<Integer> newEdgeIDs = new ArrayList<Integer>();
        newEdgeIDs.add(current);
        
        for (int i = 1; i < edgesID.size(); i++) {
            if (edgesID.get(i) != current) {
                current = edgesID.get(i);
                newEdgeIDs.add(edgesID.get(i));
            }
        }
        
        return newEdgeIDs;
    }
    
	/**
	 * Interpolate the missing edges with the shortest path. 
	 */
    List<Integer> addMissingEdges(List<Integer> edgesID) {
        List<Integer> newEdgeIDs = new ArrayList<Integer>();
        newEdgeIDs.add(edgesID.get(0));
        
        for (int i = 1; i < edgesID.size(); i++) {        
            int startNodeID = this.G.edge(edgesID.get(i-1)).to();
            int endNodeID   = this.G.edge(edgesID.get(i)).from();
            
            this.sp.searchSP(startNodeID, endNodeID);
            
            if (!sp.hasPathTo(endNodeID)) {
                System.out.println("No Path!!!");
                continue;
            }

            for (DirectedEdge edge : sp.pathTo(endNodeID)) {
                newEdgeIDs.add(edge.id());
            }

            newEdgeIDs.add(edgesID.get(i));
        }
        
        return newEdgeIDs;
    }
	
	/**
	 * Infer the travel path according to the chosen method. 
	 */
	public void infer(String inFilePath, String outFilePath) throws IOException {
        
        Constant.AlgoMode method = Constant.METHOD;
        
        // Read only a single trajectory
        List<GPSPoint> trajectory = Trajectory.readTrajectory(inFilePath);
        
		// Start finding candidates...
		findCandidatesForOneTraj(trajectory, this.gridPartition.getGridIndex());
        
        List<Integer> edgeIDs = new ArrayList<Integer>();
        
        if (method == Constant.AlgoMode.OBR_SIMP) {
            OrientedBoundingRectangle obr = new OrientedBoundingRectangle(
                this.G, this.nodes, this.streets, outFilePath, this.sp, false);
            edgeIDs = obr.computeFootprint(trajectory);
        }
        
        if (method == Constant.AlgoMode.OBR) {
            OrientedBoundingRectangle obr = new OrientedBoundingRectangle(
                this.G, this.nodes, this.streets, outFilePath, this.sp, true);
            edgeIDs = obr.computeFootprint(trajectory);
        }
        
        if (method == Constant.AlgoMode.FILTER) {
            double epsilon = Constant.NOISE_SIGMA * 1; // meters
            
            // Simplification.rectFilter(trajectories, epsilon);
            // System.out.println("FILTER is called!!!" + epsilon);
            trajectory = Simplification.rectFilter(trajectory, epsilon);
            System.out.println("simp size:" + trajectory.size());
        }
        
		if (method == Constant.AlgoMode.HMM || method == Constant.AlgoMode.FILTER) {
			// Just for HMM
			System.out.println("Start finding the best candidate...");
			vertbi(trajectory);
		}
		
        if (method != Constant.AlgoMode.OBR && method != Constant.AlgoMode.OBR_SIMP) {
            System.out.println("METHOD:" + method);
            edgeIDs = convertIterableToList(computeOptimalPath(trajectory, method));
            
            if (edgeIDs.size() != trajectory.size()) {
                System.out.println("edgeIDs.size():" + edgeIDs.size() 
                    + " trajectory.size():" + trajectory.size());
                System.out.println("Error happens, because the number"
                    + " of mapped edges is not equal to that of the"
                    + " original trajectory.");
                return;
            }
            
            // removing duplicates && add missing edges
            edgeIDs = removeDuplicates(edgeIDs);
            edgeIDs = addMissingEdges(edgeIDs);
        }
        
        System.out.println("outFilePath: " + outFilePath);
        writeEstimatedPath(edgeIDs, outFilePath);
	}
    
	/**
	 * This class is a custom comparator for sorting purpose.
	 */
	private class CandidateComparable implements Comparator<Candidate>{ 
	    @Override
	    public int compare(Candidate o1, Candidate o2) {
			if      (o1.getDistance() > o2.getDistance()) 	return 1;
			else if (o1.getDistance() < o2.getDistance())	return -1;
			else											return 0;
	    }
	}
    
	/**
	 * Run the inferring tasks for GPS trajectory data. 
	 */
	public static void runInferringTasks() throws IOException {
        
		Stopwatch timer;
		PreProcessingMap preProcessingMap = new PreProcessingMap();
        
        timer = new Stopwatch();
        
        // +++++++++++++++++++++  Read Roadmap Part +++++++++++++++++++++++++
		List<OSMNode> requiredNodes = preProcessingMap.readNodes(Constant.verticesFilepath);
		List<Street> streets = preProcessingMap.readStreets(Constant.streetsFilepath,
			requiredNodes);
        // +++++++++++++++++++++  Read Roadmap Part +++++++++++++++++++++++++
		
		// +++++++++++++++++++++  DijkstraSP Part +++++++++++++++++++++++++
		// Nodes: requiredNodes
		// Edges: streets
		EdgeWeightedDigraph G = new EdgeWeightedDigraph(
			requiredNodes.size(), streets
		);
		
		DijkstraSP sp = new DijkstraSP(G);
		// +++++++++++++++++++++  DijkstraSP Part +++++++++++++++++++++++++
		
		// +++++++++++++++++++++  Path Inference Part +++++++++++++++++++++++
		
        GridPartition gridPartition = new GridPartition(streets);
        
        PathInference pathInference = new PathInference(G, sp, requiredNodes, streets, gridPartition);
        
        System.out.println("Read map and build grids # Elapsed time: " + timer.elapsedTime());
        
        // if output directory does not exist, then create all necessary directories.
        File outDictFile = new File(Constant.OUTPUT_DIR);
        if (!outDictFile.exists()) {
            outDictFile.mkdirs();
        }

        for (int i = 0; i < Constant.TRAJ_FILE_NUM; i++) {
            System.out.println("Traj: " + i);

            timer = new Stopwatch();

            String trajFilename = Constant.TRAJ_FILE_PREFIX + i + ".txt";
            String inputFilePath = Constant.INPUT_DIR + trajFilename;
            String outputFilePath = Constant.OUTPUT_DIR + trajFilename;

            pathInference.infer(
                inputFilePath,
                outputFilePath
            );

            System.out.println("RUNTIME: Elapsed time: " + timer.elapsedTime());
        }
        // +++++++++++++++++++++  Path Inference Part +++++++++++++++++++++++
	}
    
	/**
	 * The main function. 
	 */
	public static void main(String [] args) {
        
        try {
            PathInference.runInferringTasks();
        } catch(Exception ex) {
            ex.printStackTrace();
        }
    }
}