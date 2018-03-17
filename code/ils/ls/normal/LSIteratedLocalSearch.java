package com.graphhopper.routing.ils.ls.normal;

import com.carrotsearch.hppc.IntHashSet;
import com.graphhopper.routing.AbstractRoutingAlgorithm;
import com.graphhopper.routing.DijkstraBidirectionCH;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.RoutingAlgorithm;
import com.graphhopper.routing.ils.BikePriorityWeighting;
import com.graphhopper.routing.ils.IlsAlgorithm;
import com.graphhopper.routing.ils.Iteration;
import com.graphhopper.routing.ils.ls.Arc;
import com.graphhopper.routing.ils.ls.Ellipse;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.util.TraversalMode;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.*;
import com.graphhopper.util.shapes.GHPoint;
import com.graphhopper.util.shapes.GHPoint3D;
import com.graphhopper.util.shapes.Shape;
import com.sun.istack.internal.NotNull;
import com.sun.istack.internal.Nullable;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static com.graphhopper.util.Parameters.Routing.*;

/**
 * Routing Algorithm which implements the bike route Iterated Local Search algorithm from the following paper:
 * https://dl.acm.org/citation.cfm?id=2820835
 */
public class LSIteratedLocalSearch extends AbstractRoutingAlgorithm implements ShortestPathCalculator, IlsAlgorithm {

    private final Logger logger = LoggerFactory.getLogger(getClass());

    // Constants passed in as parameters
    private final double MIN_ROAD_SCORE;
    private final int MIN_ROAD_LENGTH;
    private final double MAX_COST;
    private final int MAX_ITERATIONS;
    private final long SEED;

    private Graph CHGraph; // Graph used for CH Dijkstra search
    private EdgeFilter levelEdgeFilter; // Used for CH Dijkstra search
    private Weighting scoreWeighting; // Used for scoring arcs
    private int s, d; // Start and End Node IDs
    private Random random;
    private Iteration[] iterations; // Keep track of score at each iteration

    private boolean isFinished = false;

    ////////////////////////////////////////////
    // TEST CODE
    ////////////////////////////////////////////
    private final Mode MODE;
    private final double BUDGET_PERCENTAGE;

    private enum Mode {
        NORMAL,
        FIXED_PERCENTAGE_BUDGET,
        INCREMENTAL_BUDGET,
        NORMALIZED_SCORES;

        static Mode getMode(int value) {
            if(value >= 0 && value < values().length) {
                return values()[value];
            }
            throw new RuntimeException("Invalid mode specified!");
        }
    }

    /**
     * Creates a new ILS algorithm instance.
     *
     * @param graph           Graph to run algorithm on.
     * @param weighting       Weighting to calculate costs.
     * @param levelEdgeFilter Edge filter for CH shortest path computation
     * @param params          Parameters map.
     */
    public LSIteratedLocalSearch(Graph graph, Weighting weighting,
                                 EdgeFilter levelEdgeFilter, PMap params) {
        super(graph.getBaseGraph(), weighting, TraversalMode.EDGE_BASED_1DIR);

        CHGraph = graph;
        this.levelEdgeFilter = levelEdgeFilter;
        scoreWeighting = new BikePriorityWeighting(flagEncoder);

        MAX_COST = params.getDouble(MAX_DIST, DEFAULT_MAX_DIST);
        MAX_ITERATIONS = params.getInt(Parameters.Routing.MAX_ITERATIONS, DEFAULT_MAX_ITERATIONS);
        MIN_ROAD_SCORE = params.getDouble(Parameters.Routing.MIN_ROAD_SCORE, DEFAULT_MIN_ROAD_SCORE);
        MIN_ROAD_LENGTH = params.getInt(Parameters.Routing.MIN_ROAD_LENGTH, DEFAULT_MIN_ROAD_LENGTH);
        SEED = params.getLong(Parameters.Routing.SEED, System.currentTimeMillis());

        random = new Random(SEED);
        iterations = new Iteration[MAX_ITERATIONS];

        ////////////////////////////////////////////
        // TEST CODE
        ////////////////////////////////////////////
        MODE = Mode.getMode(params.getInt(Parameters.Routing.MODE, DEFAULT_MODE));
        BUDGET_PERCENTAGE = params.getDouble(Parameters.Routing.BUDGET_PERCENTAGE,
                Parameters.Routing.DEFAULT_BUDGET_PERCENTAGE);
        if(MODE.equals(Mode.NORMALIZED_SCORES)) {
            double SCORE_CUTOFF = params.getDouble(Parameters.Routing.SCORE_CUTOFF, DEFAULT_SCORE_CUTOFF);
            scoreWeighting = new NormalizedBikePriorityWeighting(flagEncoder, SCORE_CUTOFF);
        }

        if(params.getBool(USE_SCALED_SCORES, false)) {
            scoreWeighting = new ScaledBikePriorityWeighting(flagEncoder);
        }
    }

    /**
     * Calculates a route between the specified node IDs.
     *
     * @param from Start Node ID.
     * @param to   End Node ID.
     * @return Path
     */
    @Override
    public Path calcPath(int from, int to) {
        checkAlreadyRun();
        s = from;
        d = to;
        return runILS();
    }

    /**
     * Main algorithm loop
     */
    private Path runILS() {
        long start = System.currentTimeMillis();
        Route solution;
        if(shortestPath(s, d).getDistance() > MAX_COST) {
            solution = Route.newRoute(this, graph, weighting, scoreWeighting, s, d, MAX_COST);
        } else {
            solution = initializeSolution();
            logger.info("Seed: " + SEED);
            for(int i = 1; i <= MAX_ITERATIONS; i++) {
                double score = solution.getPath().getScore();
                logger.debug("Iteration " + i);
                List<Arc> arcRemovalPool = solution.getCandidateArcsByIP();
                logger.debug("Possible arcs to remove from solution: " + arcRemovalPool.size());

                int randomIndex = random.nextInt(arcRemovalPool.size());
                Arc arcToRemove = arcRemovalPool.remove(randomIndex);
                List<Arc> inheritedCas = arcToRemove.getCas();

                // Remaining budget after removing "arcToRemove" from solution
                double pathBudget = solution.getRemainingCost() + arcToRemove.cost;

                ////////////////////////////////////////////
                // TEST CODE
                ////////////////////////////////////////////
                if(MODE.equals(Mode.FIXED_PERCENTAGE_BUDGET)) {
                    pathBudget = pathBudget * BUDGET_PERCENTAGE;
                } else if(MODE.equals(Mode.INCREMENTAL_BUDGET)) {
                    double percent = ((double) i / MAX_ITERATIONS) * (1 - BUDGET_PERCENTAGE);
                    pathBudget = (pathBudget * percent) + BUDGET_PERCENTAGE;
                }

                Route path = generatePath(solution.getPrev(arcToRemove), solution.getNext(arcToRemove),
                        pathBudget, arcToRemove.score, inheritedCas);

                if(!path.isEmpty()) {
                    logger.debug("Found path with with dist " + path.getCost());
                    int index = solution.removeArc(arcToRemove);
                    solution.insertRoute(index, path);
                    for(Arc arc : solution) {
                        // Remaining budget after removing "arc" from solution
                        double newBudget = solution.getRemainingCost() + arc.cost;

                        int startCAS = solution.getPrev(arc);
                        int endCAS = solution.getNext(arc);

                        if(path.contains(arc) || arc.adjNode == startCAS || arc.baseNode == endCAS) {
                            // Using removed arc's CAS to compute next CAS (inherit)
                            computeCAS(arc, inheritedCas, startCAS, endCAS, newBudget);
                        } else {
                            double oldBudget = solution.getRemainingCost() + arcToRemove.cost;
                            updateCAS(arc, inheritedCas, startCAS, endCAS, newBudget, oldBudget);
                        }
                    }
                }

                long elapsed = System.currentTimeMillis() - start;
                iterations[i - 1] = new Iteration(score, elapsed / 1000.0);
            }
        }

        isFinished = true;

        return solution.getPath();
    }

    /**
     * Creates a new Route, adds a fake arc, and computes first CAS.
     *
     * @return Route.
     */
    private Route initializeSolution() {
        Route route = Route.newRoute(this, graph, weighting, scoreWeighting, s, d, MAX_COST);
        // Add fake edge to start solution
        Arc arc = new Arc(Arc.FAKE_ARC_ID, s, d, MAX_COST, 0, PointList.EMPTY);
        computeCAS(arc, null, s, d, MAX_COST);
        route.addArc(0, arc);

        return route;
    }

    /**
     * Computes the Candidate Arc Set for the specified start, end, and cost parameters.
     *
     * @param arc  Arc to set CAS on.
     * @param cas  Current CAS. May be null.
     * @param s    Start Node ID.
     * @param d    End Node Id.
     * @param cost Cost allowance.
     */
    private void computeCAS(Arc arc, @Nullable List<Arc> cas, int s, int d, double cost) {
        List<Arc> result = new ArrayList<>();

        GHPoint focus1 = new GHPoint(nodeAccess.getLatitude(s), nodeAccess.getLongitude(s));
        GHPoint focus2 = new GHPoint(nodeAccess.getLatitude(d), nodeAccess.getLongitude(d));
        Ellipse ellipse = new Ellipse(focus1, focus2, cost);

        // If we don't have a CAS yet
        // Fetch arcs from the graph using spatial indices
        if(cas == null) {
            // Since s is one of the foci of our ellipse, it will always be contained in it.
            // Use s as the node which we start our search
            cas = getAllArcs(ellipse, s);
        }

        logger.debug("Starting to compute CAS! num arcs: " + cas.size() + " cost: " + cost);

        outer:
        for(Arc e : cas) {

            // Basic restrictions on attractive arcs
            if(e.score > MIN_ROAD_SCORE && e.cost > MIN_ROAD_LENGTH) {
                // Spatial-based feasibility checking
                for(GHPoint3D ghPoint3D : e.points) {
                    if(!ellipse.contains(ghPoint3D.lat, ghPoint3D.lon)) {
                        continue outer;
                    }
                }

                // Check arc feasibility
                if(getPathCost(s, d, e) <= cost) {
                    calcQualityRatio(e, s, d);
                    result.add(e);
                }
            }
        }

        logger.debug("Finished computing CAS! size: " + result.size());

        arc.setCas(result);
    }

    /**
     * Fetches all Arcs from the graph which are contained inside of the specified Shape.
     *
     * @param shape     Shape.
     * @param startNode Node to start search from.
     * @return Arc list.
     */
    private List<Arc> getAllArcs(final Ellipse shape, int startNode) {
        logger.debug("Fetching arcs from graph!");
        final List<Arc> arcs = new ArrayList<>();

        BreadthFirstSearch bfs = new BreadthFirstSearch() {
            final Shape localShape = shape;
            final IntHashSet edgeIds = new IntHashSet();

            @Override
            protected boolean goFurther(int nodeId) {
                return localShape.contains(nodeAccess.getLatitude(nodeId), nodeAccess.getLongitude(nodeId));
            }

            @Override
            protected boolean checkAdjacent(EdgeIteratorState edge) {
                if(localShape.contains(nodeAccess.getLatitude(edge.getAdjNode()), nodeAccess.getLongitude(edge.getAdjNode()))) {
                    int edgeId = edge.getEdge();
                    if(!edgeIds.contains(edgeId)) {
                        arcs.add(getArc(edge));
                        edgeIds.add(edgeId);
                    }
                    return true;
                }
                return false;
            }
        };


        bfs.start(outEdgeExplorer, startNode);

        logger.debug("Got all arcs inside of ellipse! num: " + arcs.size());

        return arcs;
    }

    /**
     * Returns an Arc object instance from the specified EdgeIterator from the Graph.
     *
     * @param edgeIterator Edge
     * @return Arc
     */
    private Arc getArc(EdgeIteratorState edgeIterator) {
        int edge = edgeIterator.getEdge();
        int baseNode = edgeIterator.getBaseNode();
        int adjNode = edgeIterator.getAdjNode();
        double edgeCost = edgeIterator.getDistance();

        double edgeScore = scoreWeighting
                .calcWeight(edgeIterator, false, baseNode);

        return new Arc(edge, baseNode, adjNode, edgeCost, edgeScore, edgeIterator.fetchWayGeometry(0));
    }

    /**
     * Updates the Candidate Arc Set for the specified Arc.
     *
     * @param arc       Arc to update.
     * @param s         Start Node Id.
     * @param d         End Node Id.
     * @param newBudget New allowable budget.
     * @param oldBudget Old allowable budget.
     */
    private void updateCAS(@NotNull Arc arc, @NotNull List<Arc> cas, int s, int d, double newBudget, double oldBudget) {
        // Restrict CAS using inherit property
        if(newBudget < oldBudget) {
            List<Arc> newCas = new ArrayList<>();
            for(Arc e : cas) {
                // Remove any arc whose path is too big
                if(getPathCost(s, d, e) <= newBudget) {
                    newCas.add(e);
                }
            }
            arc.setCas(newCas);
        } else if(newBudget > oldBudget) {
            computeCAS(arc, null, s, d, newBudget);
        }
    }

    /**
     * Computes the Quality Ratio for the specified Arc.
     *
     * @param arc Arc.
     * @param s   Start Node ID.
     * @param d   End Node ID.
     */
    private void calcQualityRatio(@NotNull Arc arc, int s, int d) {
        Path sp1 = shortestPath(s, arc.baseNode);
        Path sp2 = shortestPath(arc.adjNode, d);

        double value = 0;

        List<EdgeIteratorState> edges = sp1.calcEdges();
        edges.addAll(sp2.calcEdges());

        for(EdgeIteratorState edge : edges) {
            value += scoreWeighting.calcWeight(edge, false, edge.getBaseNode());
        }

        value += arc.score;
        value /= (sp1.getDistance() + arc.cost + sp2.getDistance());

        if(Double.isNaN(value)) {
            value = 0;
        }

        arc.qualityRatio = value;
    }

    /**
     * Returns a list of Arcs from the specified CAS whose Quality Ratio scores are above the average.
     *
     * @param cas CAS
     * @return Arc list.
     */
    private List<Arc> getCandidateArcsByQR(List<Arc> cas) {
        List<Arc> arcs = new ArrayList<>();
        double avgQR = 0;
        for(Arc ca : cas) {
            avgQR += ca.qualityRatio;
        }
        avgQR /= cas.size();

        for(Arc ca : cas) {
            if(ca.qualityRatio >= avgQR) {
                arcs.add(ca);
            }
        }

        return arcs;
    }

    /**
     * Generates a Route from the specified start and end nodes whose distance is less than the specified budget and
     * total score is above the specified.
     *
     * @param s         Start Node Id.
     * @param d         End Node Id.
     * @param dist      Allowable budget.
     * @param minProfit Minimum required score.
     * @param cas       CAS
     * @return Route. May be empty!
     */
    private Route generatePath(int s, int d, double dist, double minProfit, List<Arc> cas) {
        logger.debug("Generating path! dist: " + dist + " minProfit: " + minProfit + " cas size: " + cas.size());
        Route route = Route.newRoute(this, graph, weighting, scoreWeighting, s, d, dist);

        List<Arc> arcs = getCandidateArcsByQR(cas);
        while(!arcs.isEmpty() && route.getCost() < dist) {
            int randomIndex = random.nextInt(arcs.size());
            Arc e = arcs.remove(randomIndex);
            route.insertArcAtMinPathSegment(e);
        }

        if(route.getScore() > minProfit) {
            return route;
        } else {
            return Route.newRoute(this, graph, weighting, scoreWeighting, s, d, dist);
        }

    }

    @Override
    public double getPathCost(int s, int d, @NotNull Arc arc) {
        return shortestPath(s, arc.baseNode).getDistance() + arc.cost +
                shortestPath(arc.adjNode, d).getDistance();
    }

    @Override
    public Path shortestPath(int s, int d) {
        RoutingAlgorithm search =
                new DijkstraBidirectionCH(CHGraph,
                        weighting, TraversalMode.NODE_BASED)
                        .setEdgeFilter(levelEdgeFilter);

        return search.calcPath(s, d);
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

    // Used for tracking progress of iterations
    @Override
    public Iteration[] getIterationInfo() {
        return iterations;
    }
}
