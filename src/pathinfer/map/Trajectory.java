package pathinfer.map;

import java.util.Scanner;
import java.util.ArrayList;
import java.io.*;
import java.util.List;
import pathinfer.main.Constant;

/**
 *  This class provides a method to read trajectory data from a file. 
 * 
 *  @author Hengfeng Li
 */
public class Trajectory {
    
    /**
     * Read a trajectory from the input file. 
     */
	public static List<GPSPoint> readTrajectory(String inFileName) throws IOException {
		// Open the trajectory file
		File file = new File(inFileName);
		Scanner scanner = new Scanner(file);
		
        String line = "";
        
        if (!Constant.TRAJ_FILE_IGNORE_HEADER) {
            // read first line
            line  = scanner.nextLine().trim();
            int numTrajs = Integer.parseInt(line);
            System.out.println("Num of trajectories:" + numTrajs);
        }
		
		List<GPSPoint> trajectory = new ArrayList<GPSPoint>();
		
		while (scanner.hasNextLine()) {
            line = scanner.nextLine().trim();
			String[] params = line.split(Constant.TRAJ_FILE_SPLIT);
			
			int veh_id   = Integer.parseInt(params[Constant.TRAJ_FILE_DRIVER_ID]);
			long timestamp = Long.parseLong(params[Constant.TRAJ_FILE_TIME]);
			double point_lat = Double.parseDouble(params[Constant.TRAJ_FILE_LAT]);
			double point_lon = Double.parseDouble(params[Constant.TRAJ_FILE_LON]);
            
			LonLat point = new LonLat(point_lon, point_lat);
			
			trajectory.add(new GPSPoint(point, timestamp, line));
		}
		
		scanner.close();
		
		return trajectory;
	}
}