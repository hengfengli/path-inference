package pathinfer.main;

import pathinfer.map.Boundary;
import pathinfer.stats.NormalDistribution;
import pathinfer.stats.ExponentialDistribution;

/**
 *  This class defines all the constants. 
 *  
 *  @author Hengfeng Li
 */
public class Constant {
    
    public static String roadmapCity = "porto";
    public static String ROADMAP_DIR = "map-data/map-" + roadmapCity + "/";
    public static String edgesFilepath = ROADMAP_DIR + "edges.txt";
    public static String verticesFilepath = ROADMAP_DIR + "vertex.txt";
    public static String streetsFilepath = ROADMAP_DIR + "streets.txt";
    
    public static Boundary boundary = new Boundary(
        -8.7270939,-8.4000143,41.0000040,41.2999941
    );
    
	public static enum AlgoMode {
	    CLOSEST, HMM, INCREMENT, OBR, FILTER, OBR_SIMP
	};
    public static AlgoMode METHOD = AlgoMode.OBR;
    
    public static String INPUT_DIR = "trips/";
    public static String TRAJ_FILE_PREFIX = "trip_";
    public static int TRAJ_FILE_NUM = 10;
    public static String TRAJ_FILE_SPLIT = ",";
    public static boolean TRAJ_FILE_IGNORE_HEADER = true;
    
    public static int TRAJ_FILE_DRIVER_ID = 0;
    public static int TRAJ_FILE_LAT = 1;
    public static int TRAJ_FILE_LON = 2;
    public static int TRAJ_FILE_TIME = 3;
    
    public static String OUTPUT_DIR = "out_trips/";
    public static int NUM_GRID_ROWS = 100;
    public static int NUM_GRID_COLS = 100;
    
    public static final int SEARCH_NUM_LIMIT = 10000;  // limit the number of search edges
    
    public final static double RECT_SIZE = 0.001;
    public final static double EPSILON = 0.000001;
    public final static double METER_TO_LONLAT = 0.000011;
    
    public final static int OBR_RECT_WIDTH_NUM = 8;
    public final static double OBR_EXTENDED_WIDTH = Constant.OBR_RECT_WIDTH_NUM 
        * Constant.NOISE_SIGMA * Constant.METER_TO_LONLAT;
    public final static double OBR_TRAJ_SPARSE_DENSITY = 0.0120480971482;
	
    public static double MAX_DIST_METERS = 300;
	public static double NOISE_SIGMA = 10;
    public static NormalDistribution norm = new NormalDistribution(0, Constant.NOISE_SIGMA);
    public static double EXP_ALPHA = norm.compute(0);
	public static double EXP_BETA = 50;
    public static int NUM_CANDIDATES = 10;
    
	public static double IMPOSSIBLE_DIST = 5000;
}