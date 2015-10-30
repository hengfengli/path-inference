package pathinfer.map;

import java.io.*;
import java.util.*;

import pathinfer.geo.*;
import pathinfer.graph.*;
import pathinfer.obr.OrientedBoundingRectangle;
import pathinfer.stats.NormalDistribution;

import pathinfer.main.Constant;

/**
 *  This class provide all the methods to read streets and nodes from files. 
 *
 *  @author Hengfeng Li
 */
public class PreProcessingMap {
    
    /**
     * Read edges from the input file. 
     */
	public List<Street> readStreets(String inFileName, List<OSMNode> nodes) 
		throws IOException {
		File file = new File(inFileName);
		Scanner scanner = new Scanner(file);
		
		// read first line
		String line  = scanner.nextLine().trim();
		int numStreets = Integer.parseInt(line);
		List<Street> streets = new ArrayList<Street>();
        System.out.println("Num of edges:" + numStreets);
		
		while (scanner.hasNextLine()) {
            line = scanner.nextLine().trim();
			String[] params = line.split(" ");
			
			int count = Integer.parseInt(params[0]);
			
			OSMNode startNode = nodes.get(Integer.parseInt(params[1]));
			OSMNode endNode   = nodes.get(Integer.parseInt(params[4]));
			
			double distance = Double.parseDouble(params[7]);
			int level = Integer.parseInt(params[8]);
			
			streets.add(new Street(count, startNode, 
				endNode, distance, level));
		}
		
		scanner.close();
		
		return streets;
	}
	
    /**
     * Read nodes from the input file. 
     */
	public List<OSMNode> readNodes(String inFileName) throws IOException {
		
		File file = new File(inFileName);
		Scanner scanner = new Scanner(file);
		
		// read first line
		String line  = scanner.nextLine().trim();
		int numNodes = Integer.parseInt(line);
		List<OSMNode> nodes = new ArrayList<OSMNode>();
        System.out.println("Num of nodes:" + numNodes);
		
		while (scanner.hasNextLine()) {
            line = scanner.nextLine().trim();
			String[] params = line.split(" ");
			
			int id     = Integer.parseInt(params[0]);
			String nodeID = params[1];
			double lon = Double.parseDouble(params[2]);
			double lat = Double.parseDouble(params[3]);
			
			OSMNode node = new OSMNode(nodeID, new LonLat(lon,lat));
			node.setIndex(id);
			
			nodes.add(node);
		}
		
		scanner.close();
		
		return nodes;
	}
}