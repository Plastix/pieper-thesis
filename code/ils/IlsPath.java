package com.graphhopper.routing.ils;

import com.graphhopper.routing.Path;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.EdgeIteratorState;

/**
 * Path object returned by an ILS routing algorithm. This classes accepts a weighting for scoring edges of the graph
 * and adds to the total score of the path when {@link #processEdge(int, int, int)} is called.
 */
public class IlsPath extends Path {

    private Weighting scoreWeighting;
    private double score;

    public IlsPath(Graph graph, Weighting costWeighting, Weighting scoreWeighting) {
        super(graph, costWeighting);
        this.scoreWeighting = scoreWeighting;
        score = 0;
    }

    @Override
    public void processEdge(int edgeId, int adjNode, int prevEdgeId) {
        super.processEdge(edgeId, adjNode, prevEdgeId);
        EdgeIteratorState edge = graph.getEdgeIteratorState(edgeId, adjNode);
        double weight = scoreWeighting.calcWeight(edge, false, prevEdgeId);
        score += weight;
    }

    public double getScore() {
        return score;
    }
}
