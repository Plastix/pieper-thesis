package com.graphhopper.routing.ils.ls.backtrack;

import com.carrotsearch.hppc.IntHashSet;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.ils.IlsPath;
import com.graphhopper.routing.ils.ls.Arc;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.EdgeIteratorState;
import com.sun.istack.internal.NotNull;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * Object which represents a path created by the {@link NOBIteratedLocalSearch}
 * algorithm.
 */
final class Route implements Iterable<Arc> {

    private final Logger logger = LoggerFactory.getLogger(getClass());

    private Graph graph;
    private Weighting timeWeighting;
    private Weighting scoreWeighting;
    private ShortestPathCalculator sp;
    private final int s, d; // Start & End Node IDs
    private final double MAX_COST;

    private List<Arc> arcs; // List of "attractive arcs" in the Route
    private List<IlsPathCh> blankSegments; // List of shortest paths connecting non-contiguous attractive arcs.
    private double cost, score; // Current

    private Route(ShortestPathCalculator shortestPathCalculator, Graph graph, Weighting timeWeighting,
                  Weighting scoreWeighting, int s, int d, double maxCost) {
        sp = shortestPathCalculator;
        arcs = new ArrayList<>();
        blankSegments = new ArrayList<>();
        cost = 0;
        score = 0;
        this.s = s;
        this.d = d;
        this.graph = graph;
        this.timeWeighting = timeWeighting;
        this.scoreWeighting = scoreWeighting;
        MAX_COST = maxCost;
    }

    /**
     * Static factory method for creating a new Route instance.
     *
     * @param sp             Interface which can calculate Shortest Paths.
     * @param graph          Graph.
     * @param weighting      Weighting used to calculate distance of added arcs.
     * @param scoreWeighting Weighting used to calculate score of added arcs.
     * @param s              Start Node ID.
     * @param d              End Node ID.
     * @return New Route Instance.
     */
    static Route newRoute(@NotNull ShortestPathCalculator sp, @NotNull Graph graph,
                          @NotNull Weighting weighting, @NotNull Weighting scoreWeighting,
                          int s, int d, double maxCost) {
        return new Route(sp, graph, weighting, scoreWeighting, s, d, maxCost);
    }

    /**
     * Adds the specified Arc to the Route at the specified index.
     * Throws {@link IndexOutOfBoundsException} if index <= 0 or index > {@link Route#length()}.
     *
     * @param index Index to insert Arc.
     * @param arc   Arc to insert.
     */
    void addArc(int index, @NotNull Arc arc) {
        int length = length();
        if(index < 0 || index > length) {
            throw new IndexOutOfBoundsException(String.format("index %d, length %d", index, length));
        }

        updatePathSegments(index, arc, arc, new IntHashSet());

        arcs.add(index, arc);
        cost += arc.cost;
        score += arc.score;
    }

    /**
     * Removes the first instance of the specified Arc from the Route.
     *
     * @param a Arc to remove.
     * @return Index of removed Arc. Returns -1 if Arc was not in the current Route.
     */
    int removeArc(@NotNull Arc a) {
        int index = arcs.indexOf(a);

        // Short circuit if Arc is not present in Route
        if(index == -1) {
            throw new IllegalArgumentException("Arc is not in route!");
        }

        // Remove two path segments surrounding Arc
        IlsPathCh segment1 = blankSegments.remove(index);
        IlsPathCh segment2 = blankSegments.remove(index);
        cost -= segment1.getDistance();
        cost -= segment2.getDistance();

        // If we have more than 1 arc we need to add a new path segment to join the Route
        int length = length();
        if(length > 1) {
            int start = s;
            int end = d;

            // Calculate start/end points for the new blank path segment
            int prevIndex = index - 1;
            if(prevIndex >= 0 && prevIndex <= length - 1) {
                start = arcs.get(prevIndex).adjNode;
            }

            int nextIndex = index + 1;
            if(nextIndex <= length - 1) {
                end = arcs.get(nextIndex).baseNode;
            }

            // Calculate and add new path segment
            IntHashSet blacklist = getArcIdSet();
            blacklist.removeAll(arcs.get(index).edgeId); // Allow the arc we are currently removing
            IlsPathCh segment = sp.shortestPath(start, end, blacklist);

            if(!segment.isFound()) {
                throw new RuntimeException("BAD THINGS");
            }
            blankSegments.add(index, segment);
            cost += segment.getDistance();
        }

        arcs.remove(index);
        cost -= a.cost;
        score -= a.score;

        return index;
    }

    /**
     * Adds the specified Route to the current Route at the specified index.
     * Throws {@link IndexOutOfBoundsException} if index <= 0 or index > {@link Route#length()}.
     *
     * @param index Index to insert Route.
     * @param route Route to insert.
     */
    void insertRoute(int index, @NotNull Route route) {
        int length = length();
        if(index < 0 || index > length) {
            throw new IndexOutOfBoundsException(String.format("index %d, length %d", index, length));
        }

        // Only add Route if it is non-empty
        if(!route.isEmpty()) {
            // We need to remove the inserted routes starting and ending path segments
            // We recalculate the new path segments below
//            IlsPathCh head = route.blankSegments.remove(0);
//            IlsPathCh tail = route.blankSegments.remove(route.blankSegments.size() - 1);
//            route.cost -= head.getDistance();
//            route.cost -= tail.getDistance();

//            Arc first = route.arcs.get(0);
//            Arc last = route.arcs.get(route.length() - 1);
//
//            updatePathSegments(index, first, last, route.getArcIdSet());

            // If non-empty, remove the previous blank path segment before inserting the new route
            if(length > 0) {
                IlsPathCh removed = blankSegments.remove(index);
                cost -= removed.getDistance();
            }

            score += route.score;
            cost += route.cost;
            arcs.addAll(index, route.arcs);
            blankSegments.addAll(index, route.blankSegments);
        }
    }

    /**
     * Updates the blank path segments at the specified index. Used when adding a new Arc to the route.
     * <p>
     * 1-2 --> 1-3-2
     *
     * @param index Index of blank path segments to update.
     * @param left  Left bound of the Arc to be inserted.
     * @param right Right bound of the Arc to be inserted.
     */
    private void updatePathSegments(int index, Arc left, Arc right, IntHashSet list) {
        int length = length();
        int start = s, end = d;

        int startIndex = index - 1;
        if(startIndex >= 0 && startIndex <= length - 1) {
            start = arcs.get(startIndex).adjNode;
        }

        if(index <= length - 1) {
            end = arcs.get(index).baseNode;
        }

        IntHashSet blacklist = getArcIdSet();
        blacklist.addAll(list);
        blacklist.addAll(left.edgeId, right.edgeId);
        IlsPathCh segment1 = sp.shortestPath(start, left.baseNode, blacklist);
        cost += segment1.getDistance();

        blacklist.addAll(segment1.getEdges());
        IlsPathCh segment2 = sp.shortestPath(right.adjNode, end, blacklist);
        cost += segment2.getDistance();

        // If non-empty, remove the previous blank path segment before inserting the two new ones
        if(length > 0) {
            IlsPathCh removed = blankSegments.remove(index);
            cost -= removed.getDistance();
        }

        if(!segment1.isFound() || !segment2.isFound()) {
            throw new RuntimeException("BAD THINGS 2");
        }

        blankSegments.add(index, segment2);
        blankSegments.add(index, segment1);
    }


    /**
     * Returns the current cost (distance) of the route in meters.
     *
     * @return Sum of edge distances in the Route.
     */
    double getCost() {
        return cost;
    }

    /**
     * Returns the total score of the route.
     *
     * @return Sum of all attractive arc scores in the Route.
     */
    double getScore() {
        return score;
    }

    /**
     * Returns the leftover budget after subtracting the current Route's cost.
     *
     * @return Remaining cost left in budget.
     */
    double getRemainingCost() {
        return MAX_COST - cost;
    }

    /**
     * Converts the Route into a Path object which GraphHopper can display on a map.
     *
     * @return Fully connected Path object
     */
    IlsPath getPath() {
        IlsPath path = new IlsPath(graph, timeWeighting, scoreWeighting);

        // If we have a fake arc return no path
        if(contains(new Arc(Arc.FAKE_ARC_ID, s, d, 0, 0, null))) {
            path.setFound(false);
            return path;
        }

        for(int i = 0; i < blankSegments.size(); i++) {
            Path blank = blankSegments.get(i);
            for(EdgeIteratorState edge : blank.calcEdges()) {
                path.processEdge(edge.getEdge(), edge.getAdjNode(), edge.getEdge());
            }

            if(i < arcs.size()) {
                Arc arc = arcs.get(i);
                path.processEdge(arc.edgeId, arc.adjNode, arc.edgeId);
            }
        }

        path.setEndNode(d)
                .setFromNode(s)
                .setFound(!isEmpty());

        logger.debug("Route dist: " + path.getDistance() + " Route score: " + path.getScore());

        return path;
    }

    private int length() {
        return arcs.size();
    }

    /**
     * Returns whether the Route has any arcs in it.
     *
     * @return True if contains arc, else false.
     */
    boolean isEmpty() {
        return length() == 0;
    }

    /**
     * Returns a list of Arcs from the Route whose Improve Potential scores are above the average.
     *
     * @return Arc list.
     */
    List<Arc> getCandidateArcsByIP() {
        List<Arc> result = new ArrayList<>();
        double avgIP = 0;
        for(Arc ca : arcs) {
            calcImprovePotential(ca);
            avgIP += ca.improvePotential;
        }
        avgIP /= arcs.size();

        for(Arc ca : arcs) {
            if(ca.improvePotential >= avgIP) {
                result.add(ca);
            }
        }

        return result;
    }

    /**
     * Calculates the Improve Potential score of a given arc.
     *
     * @param arc Arc to calculate
     */
    private void calcImprovePotential(@NotNull Arc arc) {
        int v1 = getPrev(arc);
        int v2 = getNext(arc);

        double score = 0;
        double maxDist = 0;

        double dist = sp.getPathCost(v1, v2, arc, this);

        for(Arc e : arc.getCas()) {
            score += e.score - arc.score;
            maxDist = Math.max(maxDist, sp.getPathCost(v1, v2, e, this));
        }

        double result = score / (maxDist - dist);

        // Hacky fix for NaN values
        if(Double.isNaN(result) || result < 0) {
            result = 0;
        }

        arc.improvePotential = result;
    }

    /**
     * Returns the Node ID before the specified Arc in the Route.
     *
     * @param a Arc
     * @return Node ID
     */
    int getPrev(@NotNull Arc a) {
        if(!contains(a)) {
            throw new IllegalArgumentException("Arc is not in route!");
        }

        int index = arcs.indexOf(a);
        return (index - 1 >= 0) ? arcs.get(index - 1).adjNode : s;
    }

    /**
     * Returns the Node ID after the specified Arc in the Route.
     *
     * @param a Arc
     * @return Node ID.
     */
    int getNext(@NotNull Arc a) {
        if(!contains(a)) {
            throw new IllegalArgumentException("Arc is not in route!");
        }

        int index = arcs.indexOf(a);
        return (index + 1 <= length() - 1) ? arcs.get(index + 1).baseNode : d;
    }

    /**
     * Returns whether the specified Arc is in the Route.
     *
     * @param a Arc to query
     * @return True if arc is in Route, else false.
     */
    boolean contains(@NotNull Arc a) {
        return arcs.contains(a);
    }

    /**
     * Adds the specified arc to the Route at the smallest blank path segment as long as it does not go over the
     * specified budget.
     *
     * @param arc Arc to insert.
     */
    void insertArcAtMinPathSegment(@NotNull Arc arc) {
        // We have at least 1 arc and 2 blank path segments
        if(!isEmpty()) {
            int pathIndex = -1;
            double minPathValue = Double.MAX_VALUE;
            // Find smallest blank path segment
            for(int i = 0; i < blankSegments.size(); i++) {
                double value = blankSegments.get(i).getDistance();
                if(value < minPathValue) {
                    minPathValue = value;
                    pathIndex = i;
                }
            }

            int start = pathIndex == 0 ? s : arcs.get(pathIndex - 1).adjNode;
            int end = pathIndex == length() ? d : arcs.get(pathIndex).baseNode;

            double pathCost = sp.getPathCost(start, end, arc, this);
            double limit = getRemainingCost() + minPathValue;
            if(pathCost <= limit) {
                addArc(pathIndex, arc);
            }

        } else if(sp.getPathCost(s, d, arc, this) <= getRemainingCost()) {
            addArc(0, arc);
        }
    }

    public IntHashSet getArcIdSet() {
        IntHashSet results = new IntHashSet();
        for(int i = 0; i < blankSegments.size(); i++) {
            results.addAll(blankSegments.get(i).getEdges());
            if(i < arcs.size()) {
                results.add(arcs.get(i).edgeId);
            }
        }
        return results;
    }

    @Override
    public Iterator<Arc> iterator() {
        return arcs.iterator();
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder();
        builder.append("[");
        builder.append(s);
        builder.append("] -> ");

        for(int i = 0; i < blankSegments.size(); i++) {

            for(EdgeIteratorState edgeIteratorState : blankSegments.get(i).calcEdges()) {
                builder.append(edgeIteratorState.getEdge());
                builder.append("->");
            }

            if(i < arcs.size()) {
                builder.append("(");
                builder.append(arcs.get(i).edgeId);
                builder.append(")");
            }

            builder.append(" -> ");
        }

        builder.append(" [");
        builder.append(d);
        builder.append("]");
        return builder.toString();
    }
}
