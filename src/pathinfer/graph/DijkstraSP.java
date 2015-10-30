package pathinfer.graph;

import java.util.HashMap;
import java.util.Map;

import pathinfer.main.Constant;

import edu.princeton.cs.algs4.Stack;
import edu.princeton.cs.algs4.IndexMinPQ;

/**
 *  This class provides the Dijkstra Shortest Path algorithm. 
 * 
 *  Reference: http://algs4.cs.princeton.edu/code/
 *  
 */
public class DijkstraSP
{
	private Map<Integer, DirectedEdge> edgeTo;
	private Map<Integer, Double> distTo;
	private IndexMinPQ<Double> pq;
	private EdgeWeightedDigraph G;
	
	public DijkstraSP(EdgeWeightedDigraph G)
	{
		this.G = G;
		this.edgeTo = new HashMap<Integer, DirectedEdge>();
		this.distTo = new HashMap<Integer, Double>();
		this.pq = new IndexMinPQ<Double>(G.V());
	}
	
	public void searchSP(int s, int t)
	{
		clear();
		int count = 0;
		
		distTo.put(s, 0.0);
		
		pq.insert(s, 0.0);
		
		while (!pq.isEmpty()) {
			int v = pq.delMin();
			relax(G, v);
			
			if (v == t || count > Constant.SEARCH_NUM_LIMIT) break;
			count++;
		}
	}
	
	private void clear() 
	{
		this.pq = new IndexMinPQ<Double>(G.V());
		edgeTo.clear();
		distTo.clear();
	}
	
	private void relax(EdgeWeightedDigraph G, int v)
	{
		for (DirectedEdge e : G.adj(v))
		{
			int w = e.to();
			
			// If changes the value of dictionary, 
			// this variable should not be refered any more.
			Double distToW = distTo.get(w);
			if (distToW == null) distToW = Double.POSITIVE_INFINITY;
			Double distToV = distTo.get(v);
			if (distToV == null) distToV = Double.POSITIVE_INFINITY;
			
			if (distToW > distToV + e.weight())
			{
				Double updatedDestToW = distToV + e.weight();
				// updated, so distToW's value is different from distTo.get(w).
				distTo.put(w, updatedDestToW);
				edgeTo.put(w, e);
				if (pq.contains(w)) pq.change(w, updatedDestToW);
				else                pq.insert(w, updatedDestToW);
			}
		}
	}
	
	public double distTo(int v) 
	{	return distTo.get(v); 	}
	
	public boolean hasPathTo(int v) 
	{	return distTo.get(v) != null;	}
	
	public Iterable<DirectedEdge> pathTo(int v) 
	{
		if (!hasPathTo(v)) return null;
		Stack<DirectedEdge> path = new Stack<DirectedEdge>();
		for (DirectedEdge e = edgeTo.get(v); e != null; e = edgeTo.get(e.from()))
			path.push(e);
		return path;
	}
	
	public static void printPath(DijkstraSP sp, int s, int t) 
	{
        if (sp.hasPathTo(t)) {
            System.out.printf("%d to %d (%.2f)  ", s, t, sp.distTo(t));
			
            for (DirectedEdge e : sp.pathTo(t)) {
            	System.out.print(e + "   ");
            }
            System.out.println();
        }
        else {
            System.out.printf("%d to %d         no path\n", s, t);
		}
	}
}