package pathinfer.map;

import pathinfer.geo.*;
import pathinfer.main.Constant;

import java.util.List;
import java.util.ArrayList;


/**
 *  This class provides all the methods to build index of grid implementation,
 *  which reduces the search space (streets). So in each grid, we can find 
 *  out the candidates of mapping GPS point. 
 *  
 *  @author Hengfeng Li
 */
public class GridPartition {
    
	private Boundary boundary;
	private double gridWidth, gridHeight;
	private int numRows, numCols;
    private List<List<Street>> gridIndex;
    
    /**
     * Constructor. 
     */
	public GridPartition(List<Street> streets) {
		this.boundary = Constant.boundary;
		this.numRows = Constant.NUM_GRID_ROWS;
		this.numCols = Constant.NUM_GRID_COLS;
				
		this.gridWidth  = boundary.width()/this.numCols;
		this.gridHeight = boundary.height()/this.numRows;
        
        buildGridIndex(streets);
	}
    
    public List<List<Street>> getGridIndex() {
        return this.gridIndex;
    }
    
	public int xIndex(double x) {
		return (int)((x - this.boundary.minX())/this.gridWidth);
	}
	
	public int yIndex(double y) {
		return (int)((this.boundary.maxY() - y)/this.gridHeight);
	}
	
    /**
     * Find all grids that overlap the specified area. 
     */
	public List<Integer> findCoveringGrids(LonLat startnode, LonLat endnode) {
		int x1 = xIndex(startnode.getLon());
		int y1 = yIndex(startnode.getLat());
		int x2 = xIndex(endnode.getLon());
		int y2 = yIndex(endnode.getLat());
		
		List<Integer> grids = new ArrayList<Integer>();
				
		int maxX = Math.max(x1, x2);
		int maxY = Math.max(y1, y2);
		int minX = Math.min(x1, x2);
		int minY = Math.min(y1, y2);
		
		if (maxX > this.numCols-1) maxX = this.numCols-1;
		if (maxY > this.numRows-1) maxY = this.numRows-1;
		if (minX < 0) minX = 0;
		if (minY < 0) minY = 0;
		
		int spanX = maxX - minX + 1;
		int spanY = maxY - minY + 1;
		
		int x, y;

		for (int i = 0; i < spanX; i++) {
			x = minX + i;
			for (int j = 0; j < spanY; j++) {
				y = minY + j;
				grids.add( y * this.numCols + x );
			}
		}
		
		return grids;
	}
    
    /**
     * Build grid indexes for streets. 
     */
	public void buildGridIndex(List<Street> streets) {		
		// Initialize grid index as a one-dimension list. 
		this.gridIndex = new ArrayList<List<Street>>();
		int size = this.numRows * this.numCols;
		
		for (int i = 0; i < size; i++) {
			this.gridIndex.add(new ArrayList<Street>());
		}
		
		int x, y;
		double gridLon, gridLat;
		Boundary grid_boundary;
        
		// Build indexes to find grids passed by each street
		for (int i = 0; i < streets.size(); i++) {

			Street street = streets.get(i);
			
			LonLat startnode = street.getStartNode().getLonLat();
			LonLat endnode   = street.getEndNode().getLonLat();
			
			// If 'coveredGridIndexes' is empty, it means that this point
			// is out of the range. 
			List<Integer> coveredGridIndexes = findCoveringGrids(startnode, endnode);
			
			// Iterate all grids
			for (Integer index : coveredGridIndexes) {
				
				// Retrive one grid
				List<Street> grid = this.gridIndex.get(index);
				x = index%this.numCols;
				y = (int)index/this.numCols;
				
				gridLon = x * gridWidth + boundary.minX();

				gridLat = boundary.maxY() - y * gridHeight;
                
				// Check whether the street intersects with this grid.
				// represent a rectangle by using minlon, minlat, width, height
				boolean isIntersected = GeoCalculator.isLineIntersectRect(
						startnode.getLon(), startnode.getLat(), 
						endnode.getLon(), endnode.getLat(),
						gridLon, gridLat-gridHeight, gridWidth, gridHeight
				);
				
				if (isIntersected) {
					grid.add(street);
				}
			}
		}
	}
}