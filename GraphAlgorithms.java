import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Queue;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.HashMap;
import java.util.PriorityQueue;

/**
 * Implementation of various different graph algorithms.
 *
 * @author Mackenzie Williams
 * @version 1.0
 */
public class GraphAlgorithms {

    /**
     * Performs a breadth first search (bfs) on the input graph, starting at
     * the parameterized starting vertex.
     *
     * When exploring a vertex, explore in the order of neighbors returned by
     * the adjacency list. Failure to do so may cause you to lose points.
     *
     * You may import/use java.util.Set, java.util.List, java.util.Queue, and
     * any classes that implement the aforementioned interfaces, as long as they
     * are efficient.
     *
     * The only instance of java.util.Map that you may use is the
     * adjacency list from graph. DO NOT create new instances of Map
     * for BFS (storing the adjacency list in a variable is fine).
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * @param <T>   the generic typing of the data
     * @param start the vertex to begin the bfs on
     * @param graph the graph to search through
     * @return list of vertices in visited order
     * @throws IllegalArgumentException if any input is null, or if start
     *                                  doesn't exist in the graph
     */
    public static <T> List<Vertex<T>> bfs(Vertex<T> start, Graph<T> graph) {
        if (start == null || graph == null) {
            throw new IllegalArgumentException("Cannot to a BFS with null arguments");
        }
        Set<Vertex<T>> vertices = graph.getVertices();
        if (!vertices.contains(start)) {
            throw new IllegalArgumentException("Cannot perform BFS on vertex not found in the graph");
        }
        List<Vertex<T>> bfs = new LinkedList<Vertex<T>>();
        bfs.add(start);
        HashSet<Vertex<T>> vSet = new HashSet<Vertex<T>>();
        Queue<Vertex<T>> queue = new LinkedList<>();
        queue.add(start);
        Map<Vertex<T>, List<VertexDistance<T>>> adjListAll = graph.getAdjList();
        while (!queue.isEmpty()) {
            Vertex<T> curr = queue.remove();
            List<VertexDistance<T>> adjList = adjListAll.get(curr);
            vSet.add(curr);
            for (VertexDistance<T> distance : adjList) {
                if (distance.getDistance() == 0) {
                    if (!vSet.contains(distance.getVertex())) {
                        vSet.add(distance.getVertex());
                        queue.add(distance.getVertex());
                        bfs.add(distance.getVertex());
                    }
                }
            }
        }
        return bfs;
    }

    /**
     * Performs a depth first search (dfs) on the input graph, starting at
     * the parameterized starting vertex.
     *
     * When exploring a vertex, explore in the order of neighbors returned by
     * the adjacency list. Failure to do so may cause you to lose points.
     *
     * *NOTE* You MUST implement this method recursively, or else you will lose
     * all points for this method.
     *
     * You may import/use java.util.Set, java.util.List, and
     * any classes that implement the aforementioned interfaces, as long as they
     * are efficient.
     *
     * The only instance of java.util.Map that you may use is the
     * adjacency list from graph. DO NOT create new instances of Map
     * for DFS (storing the adjacency list in a variable is fine).
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * @param <T>   the generic typing of the data
     * @param start the vertex to begin the dfs on
     * @param graph the graph to search through
     * @return list of vertices in visited order
     * @throws IllegalArgumentException if any input is null, or if start
     *                                  doesn't exist in the graph
     */
    public static <T> List<Vertex<T>> dfs(Vertex<T> start, Graph<T> graph) {
        if (start == null || graph == null) {
            throw new IllegalArgumentException("Cannot perform DFS with null arguments");
        }
        Set<Vertex<T>> vertices = graph.getVertices();
        if (!vertices.contains(start)) {
            throw new IllegalArgumentException("Cannot perform DFS on vertex not found in the graph");
        }
        List<Vertex<T>> dfs = new LinkedList<>();
        HashSet<Vertex<T>> vSet = new HashSet<Vertex<T>>();
        Map<Vertex<T>, List<VertexDistance<T>>> adjListAll = graph.getAdjList();
        dfs.add(start);
        vSet.add(start);
        dfsHelper(start, dfs, vSet, adjListAll); //the recursive method
        return dfs;
    }

    /**
     * The recursive helper method for dfs.
     * Performs the search and adds to the list returned by dfs.
     *
     * @param <T>   the generic typing of the data
     * @param curr the current vertex we are looking to add
     * @param dfs the list of vertices created by the dfs
     * @param vSet the set of visisted vertices
     * @param allAdj the adjacency list of all vertex connections in the graph
     */
    private static <T> void dfsHelper(Vertex<T> curr, List<Vertex<T>> dfs, HashSet<Vertex<T>> vSet,
                               Map<Vertex<T>, List<VertexDistance<T>>> allAdj) {
        List<VertexDistance<T>> adjList = allAdj.get(curr);
        for (VertexDistance<T> distance : adjList) {
            if (distance.getDistance() == 0) {
                if (!vSet.contains(distance.getVertex())) {
                    vSet.add(distance.getVertex());
                    dfs.add(distance.getVertex());
                    dfsHelper(distance.getVertex(), dfs, vSet, allAdj);
                }
            }
        }
    }

    /**
     * Finds the single-source shortest distance between the start vertex and
     * all vertices given a weighted graph (you may assume non-negative edge
     * weights).
     *
     * Return a map of the shortest distances such that the key of each entry
     * is a node in the graph and the value for the key is the shortest distance
     * to that node from start, or Integer.MAX_VALUE (representing
     * infinity) if no path exists.
     *
     * You may import/use java.util.PriorityQueue,
     * java.util.Map, and java.util.Set and any class that
     * implements the aforementioned interfaces, as long as your use of it
     * is efficient as possible.
     *
     * You should implement the version of Dijkstra's where you use two
     * termination conditions in conjunction.
     *
     * 1) Check if all of the vertices have been visited.
     * 2) Check if the PQ is empty.
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * @param <T>   the generic typing of the data
     * @param start the vertex to begin the Dijkstra's on (source)
     * @param graph the graph we are applying Dijkstra's to
     * @return a map of the shortest distances from start to every
     * other node in the graph
     * @throws IllegalArgumentException if any input is null, or if start
     *                                  doesn't exist in the graph.
     */
    public static <T> Map<Vertex<T>, Integer> dijkstras(Vertex<T> start,
                                                        Graph<T> graph) {
        if (start == null || graph == null) {
            throw new IllegalArgumentException("Cannot perform Dijkstras with null arguments");
        }
        Set<Vertex<T>> vertices = graph.getVertices();
        if (!vertices.contains(start)) {
            throw new IllegalArgumentException("Cannot perform Dijkstras on vertex not found in the graph");
        }

        HashSet<Vertex<T>> vSet = new HashSet<Vertex<T>>();
        HashMap<Vertex<T>, Integer> distanceMap = new HashMap<Vertex<T>, Integer>();
        PriorityQueue<VertexDistance<T>> pq = new PriorityQueue<VertexDistance<T>>();
        for (Vertex<T> curr : graph.getVertices()) {
            distanceMap.put(curr, Integer.MAX_VALUE);
        }
        distanceMap.put(start, 0);
        pq.add(new VertexDistance<>(start, 0));
        Map<Vertex<T>, List<VertexDistance<T>>> adjListAll = graph.getAdjList();
        while (!pq.isEmpty() && vSet.size() < distanceMap.size()) {
            VertexDistance<T> curr = pq.remove();
            if (!vSet.contains(curr.getVertex())) {
                vSet.add(curr.getVertex());
                distanceMap.put(curr.getVertex(), curr.getDistance());
                List<VertexDistance<T>> adjList = adjListAll.get(curr.getVertex());
                for (VertexDistance<T> distance : adjList) {
                    if (!vSet.contains(distance.getVertex())) {
                        pq.add(new VertexDistance<>(distance.getVertex(), curr.getDistance()
                                + distance.getDistance()));
                    }
                }
            }
        }
        return distanceMap;
    }

    /**
     * Runs Prim's algorithm on the given graph and returns the Minimum
     * Spanning Tree (MST) in the form of a set of Edges. If the graph is
     * disconnected and therefore no valid MST exists, return null.
     *
     * You may assume that the passed in graph is undirected. In this framework,
     * this means that if (u, v, 3) is in the graph, then the opposite edge
     * (v, u, 3) will also be in the graph, though as a separate Edge object.
     *
     * The returned set of edges should form an undirected graph. This means
     * that every time you add an edge to your return set, you should add the
     * reverse edge to the set as well. This is for testing purposes. This
     * reverse edge does not need to be the one from the graph itself; you can
     * just make a new edge object representing the reverse edge.
     *
     * You may assume that there will only be one valid MST that can be formed.
     *
     * You should NOT allow self-loops or parallel edges in the MST.
     *
     * You may import/use PriorityQueue, java.util.Set, and any class that 
     * implements the aforementioned interface.
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * The only instance of java.util.Map that you may use is the
     * adjacency list from graph. DO NOT create new instances of Map
     * for this method (storing the adjacency list in a variable is fine).
     *
     * @param <T> the generic typing of the data
     * @param start the vertex to begin Prims on
     * @param graph the graph we are applying Prims to
     * @return the MST of the graph or null if there is no valid MST
     * @throws IllegalArgumentException if any input is null, or if start
     *                                  doesn't exist in the graph.
     */
    public static <T> Set<Edge<T>> prims(Vertex<T> start, Graph<T> graph) {
        if (start == null || graph == null) {
            throw new IllegalArgumentException("Cannot perform Dijkstras with null arguments");
        }
        Set<Vertex<T>> vertices = graph.getVertices();
        if (!vertices.contains(start)) {
            throw new IllegalArgumentException("Cannot perform Dijkstras on vertex not found in the graph");
        }

        HashSet<Vertex<T>> vSet = new HashSet<Vertex<T>>();
        HashSet<Edge<T>> edgeSet = new HashSet<Edge<T>>();
        PriorityQueue<Edge<T>> pq = new PriorityQueue<Edge<T>>();
        Set<Edge<T>> edges = graph.getEdges();
        Map<Vertex<T>, List<VertexDistance<T>>> adjListAll = graph.getAdjList();
        List<VertexDistance<T>> adjList = adjListAll.get(start);
        for (VertexDistance<T> distance : adjList) {
            pq.add(new Edge<T>(start, distance.getVertex(), distance.getDistance()));
        }
        vSet.add(start);
        while (!pq.isEmpty() && vSet.size() < edges.size()) {
            Edge<T> curr = pq.remove();
            if (!vSet.contains(curr.getV())) {
                vSet.add(curr.getV());
                edgeSet.add(curr);
                edgeSet.add(new Edge<T>(curr.getV(), curr.getU(), curr.getWeight()));
                adjList = adjListAll.get(curr.getV());
                for (VertexDistance<T> distance : adjList) {
                    if (!vSet.contains(distance.getVertex())) {
                        pq.add(new Edge<T>(curr.getV(), distance.getVertex(), distance.getDistance()));
                    }
                }
            }
        }
        if (vSet.size() < graph.getVertices().size()) {
            return null;
        }
        return edgeSet;
    }
}