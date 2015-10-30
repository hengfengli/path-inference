package pathinfer.graph;

/**
 *  This class defines the weighted edge in a directed graph. 
 * 
 *  Reference: http://algs4.cs.princeton.edu/code/
 *  
 */
public class DirectedEdge
{
	private final int id;
	private final int v;			// edge source
	private final int w;			// edge target
	private final double weight; 	// edge weight
	
	public DirectedEdge(int id, int v, int w, double weight)
	{
		this.id = id;
		this.v = v;
		this.w = w;
		this.weight = weight;
	}
	
	public double weight()
	{	return weight;	}
	
	public int from()
	{	return v;	}
	
	public int to()
	{	return w;	}
	
	public String toString()
	{	
		return String.format("%d:%d->%d %.2f", id, v, w, weight);
	}
	
	public int id()
	{	return id;	}
}