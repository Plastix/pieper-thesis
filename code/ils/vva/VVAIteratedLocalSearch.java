package com.graphhopper.routing.ils.vva;

import com.graphhopper.routing.AbstractRoutingAlgorithm;
import com.graphhopper.routing.DijkstraBidirectionCH;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.RoutingAlgorithm;
import com.graphhopper.routing.ils.BikePriorityWeighting;
import com.graphhopper.routing.ils.IlsAlgorithm;
import com.graphhopper.routing.ils.IlsPath;
import com.graphhopper.routing.ils.Iteration;
import com.graphhopper.routing.util.DefaultEdgeFilter;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.util.TraversalMode;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIterator;
import com.graphhopper.util.PMap;
import com.graphhopper.util.Parameters;

import static com.graphhopper.util.Parameters.Routing.*;

/**
 * Routing Algorithm which implements the bike route Iterated Local Search algorithm from the following paper:
 * https://www.sciencedirect.com/science/article/pii/S1366554514000751
 */
public class VVAIteratedLocalSearch extends AbstractRoutingAlgorithm implements IlsAlgorithm {

    private final double MAX_COST;
    private final double MIN_COST;
    private final int MAX_DEPTH;
    private final int MAX_ITERATIONS;

    private Graph CHGraph; // CH Dijkstra search
    private EdgeFilter levelEdgeFilter; // Used for CH Dijkstra search
    private Weighting scoreWeighting;

    private boolean isFinished = false;
    private int s, d;
    private Iteration[] iterations;
    private EdgeFilter bikeEdgeFilter;

    /**
     * @param graph specifies the graph where this algorithm will run on
     */
    public VVAIteratedLocalSearch(Graph graph, Weighting weighting,
                                  EdgeFilter levelEdgeFilter, PMap params) {
        super(graph.getBaseGraph(), weighting, TraversalMode.EDGE_BASED_1DIR);

        CHGraph = graph;
        this.levelEdgeFilter = levelEdgeFilter;
        scoreWeighting = new BikePriorityWeighting(flagEncoder);
        bikeEdgeFilter = new DefaultEdgeFilter(flagEncoder);

        MAX_COST = params.getDouble(MAX_DIST, DEFAULT_MAX_DIST);
        MIN_COST = params.getDouble(MIN_DIST, DEFAULT_MIN_DIST);
        MAX_DEPTH = params.getInt(SEARCH_DEPTH, DEFAULT_SEARCH_DEPTH);
        MAX_ITERATIONS = params.getInt(Parameters.Routing.MAX_ITERATIONS, DEFAULT_MAX_ITERATIONS);

        iterations = new Iteration[MAX_ITERATIONS];
    }

    @Override
    public Path calcPath(int from, int to) {
        checkAlreadyRun();
        s = from;
        d = to;
        return runILS();
    }

    private Path runILS() {
        Route solution = initialize();
        solution = improve(solution);
        isFinished = true;
        return getPath(solution);
    }

    private IlsPath getPath(Route solution) {
        return solution.getPath(graph, weighting, scoreWeighting, s, d);
    }

    private Route improve(Route solution) {
        long start = System.currentTimeMillis();
        Route newPath = new Route();
        int a = 1, r = 1, count = 0;
        while(count < MAX_ITERATIONS) {
            double score = getPath(solution).getScore();
            Route temp = solution.copy();
            int size = temp.length();

            if(r > size) {
                r = 1;
            }

            if(a + r > size - 1) {
                r = size - 1 - a;
            }

            // Remove arcs a - r
            double minScore = 0;
            int startId = s, endId = d;
            for(int i = 0; i < r; i++) {
                Arc arc = temp.removeEdgeIndex(a - 1);
                minScore += arc.score;

                if(i == 0) {
                    startId = arc.baseNode;
                }

                if(i == r - 1) {
                    endId = arc.adjNode;
                }
            }

            // Don't allow search to traverse roads already in our path
            newPath.blacklist(temp);
            if(localSearch(newPath, startId, endId, MAX_COST - temp.getCost(),
                    minScore, MAX_DEPTH)) {
                temp.insertRoute(newPath, a - 1);
                solution = temp;
                a = 1;
                r = 1;
            } else {
                a++;
                r++;
            }

            long elapsed = System.currentTimeMillis() - start;
            iterations[count] = new Iteration(score, elapsed / 1000.0);

            // Clear temp path so we can use it again
            newPath.clear();
            count++;
        }

        return solution;
    }

    private Route initialize() {
        Route route = new Route();

        if(!localSearch(route, s, d, MAX_COST, 0, MAX_DEPTH)) {
            route.clear();
        }

        return route;
    }

    private boolean localSearch(Route route, int s, int d, double dist,
                                double minProfit, int maxDepth) {
        if(maxDepth == 0) {
            return false;
        }

        // Using edgeExplorer from baseGraph for traversal (non-CH version)
        EdgeExplorer explorer = graph.createEdgeExplorer(bikeEdgeFilter);
        EdgeIterator edgeIterator = explorer.setBaseNode(s);

        while(edgeIterator.next()) {
            int currentEdge = edgeIterator.getEdge();

            if(route.containsEdge(currentEdge)) {
                continue;
            }

            double edgeCost = edgeIterator.getDistance();
            int nextNode = edgeIterator.getAdjNode();

            double remainingDist = dist - edgeCost;
            double shortestDist = shortestPath(nextNode, d);

            if(shortestDist >= remainingDist) {
                continue;
            }

            double edgeScore = scoreWeighting
                    .calcWeight(edgeIterator, false, nextNode);

            route.addEdge(currentEdge, s, nextNode, edgeCost, edgeScore);

            if(nextNode == d &&
                    route.getCost() >= MIN_COST &&
                    route.getScore() > minProfit) {
                return true;
            } else if(localSearch(route, nextNode, d, remainingDist,
                    minProfit, maxDepth - 1)) {
                return true;
            }

            route.removeEdge(currentEdge);
        }

        return false;
    }

    /**
     * Returns the shortest distance in meters between two nodes of the graph.
     */
    private double shortestPath(int s, int d) {
        RoutingAlgorithm search =
                new DijkstraBidirectionCH(CHGraph,
                        weighting, TraversalMode.NODE_BASED)
                        .setEdgeFilter(levelEdgeFilter);

        Path path = search.calcPath(s, d);
        return path.getDistance();
    }

    // Unused
    @Override
    public int getVisitedNodes() {
        return 0;
    }

    @Override
    protected boolean finished() {
        return isFinished;
    }

    // Unused
    @Override
    protected Path extractPath() {
        return null;
    }

    @Override
    public Iteration[] getIterationInfo() {
        return iterations;
    }
}
