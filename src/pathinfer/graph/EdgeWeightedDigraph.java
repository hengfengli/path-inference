package pathinfer.graph;

import java.util.List;
import pathinfer.map.*;
import pathinfer.geo.*;

import edu.princeton.cs.algs4.Bag;

/**
 *  This class provides the weighted directed graph. 
 * 
 *  Reference: http://algs4.cs.princeton.edu/code/
 *  
 */
public class EdgeWeightedDigraph
{
	private final int V;				// number of vertices
	private int E;						// number of edges
	private Bag<DirectedEdge>[] adj;	// adjacency lists
	private DirectedEdge[] edges;
	
	public EdgeWeightedDigraph(int V)
	{
		this.V = V;
		this.E = 0;
		adj = (Bag<DirectedEdge>[]) new Bag[V];
		for (int v = 0; v < V; v++)
			adj[v] = new Bag<DirectedEdge>();
	}
	
	public EdgeWeightedDigraph(int V, List<Street> streets) {
		this(V);
		
		edges = new DirectedEdge[streets.size()];
		
		// build directed graph
		for (Street street: streets) {
			// [id, source, destination, weight]
            int id = street.getStreetID();
            int v  = street.getStartNode().getIndex();
            int w  = street.getEndNode().getIndex();
            double weight = street.getDistance();
            
            // for directed graph
            DirectedEdge e = new DirectedEdge(id, v, w, weight);
            addEdge(e);
            edges[id] = e;
            
            // // for undirected graph
            // // create two directed ediges
            // DirectedEdge e1 = new DirectedEdge(id, v, w, weight);
            // DirectedEdge e2 = new DirectedEdge(id, w, v, weight);
            // 
            // addEdge(e1);
            // addEdge(e2);
            // 
            // edges[id] = e1;
		}
	}
	
	public int V() { return V; }
	public int E() { return E; }
	
	public void addEdge(DirectedEdge e)
	{
		adj[e.from()].add(e);
		E++;
	}
	
	public Iterable<DirectedEdge> adj(int v)
	{ return adj[v]; }
	
	public Iterable<DirectedEdge> edges()
	{
		Bag<DirectedEdge> bag = new Bag<DirectedEdge>();
		for (int v = 0; v < V; v++)
			for (DirectedEdge e : adj[v])
				bag.add(e);
		return bag;
	}
	
	public DirectedEdge edge(int id)
	{	return edges[id];	}
}