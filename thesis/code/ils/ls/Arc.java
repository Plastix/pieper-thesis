package com.graphhopper.routing.ils.ls;

import com.graphhopper.util.PointList;

import java.util.ArrayList;
import java.util.List;

/**
 * Class which contains metadata about a particular edge in the graph. Used by
 * the ILS algorithms.
 * <p>
 * In the ILS-CAS algorithm this represents an "attractive arc".
 */
public class Arc {
    public static final int FAKE_ARC_ID = -1;

    public final int edgeId, baseNode, adjNode;
    public final double cost, score;
    public final PointList points; // Points along the arc

    public double improvePotential, qualityRatio; // Metrics used by ILS algorithm
    private List<Arc> cas; // Candidate Arc Set of this arc

    /**
     * Constructor for creating a new Arc object.
     *
     * @param edgeId   The ID of the current edge in the graph.
     * @param baseNode The node ID of the first node which this arc connects.
     * @param adjNode  The node ID of the second nod which this arc connects.
     * @param cost     The distance of the road, in meters.
     * @param score    The score of the arc.
     * @param points   Points on the map of the arc.
     */
    public Arc(int edgeId, int baseNode, int adjNode, double cost, double score, PointList points) {
        this.edgeId = edgeId;
        this.baseNode = baseNode;
        this.adjNode = adjNode;
        this.cost = cost;
        this.score = score;
        this.points = points;
        improvePotential = -1;
        qualityRatio = -1;
        cas = new ArrayList<>();
    }

    @Override
    public String toString() {
        return "Arc{" +
                "edgeId=" + edgeId +
                '}';
    }

    /**
     * Gets the Candidate Arc Set of the current Arc.
     *
     * @return CAS
     */
    public List<Arc> getCas() {
        return cas;
    }

    /**
     * Updates the Candidate Arc Set of the current Arc.
     *
     * @param cas Candidate Arc Set to update.
     */
    public void setCas(List<Arc> cas) {
        this.cas = cas;
    }

    @Override
    public boolean equals(Object o) {
        if(this == o) return true;
        if(o == null || getClass() != o.getClass()) return false;

        Arc arc = (Arc) o;

        return edgeId == arc.edgeId && baseNode == arc.baseNode && adjNode == arc.adjNode;
    }

    @Override
    public int hashCode() {
        int result = edgeId;
        result = 31 * result + baseNode;
        result = 31 * result + adjNode;
        return result;
    }
}
