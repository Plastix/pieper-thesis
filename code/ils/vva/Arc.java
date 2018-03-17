package com.graphhopper.routing.ils.vva;

import java.util.Objects;

/**
 * Class which contains metadata about a particular edge in the graph. Used by
 * {@link Route} and {@link VVAIteratedLocalSearch}
 */
class Arc {
    final int edgeId, baseNode, adjNode;
    final double cost, score;

    Arc(int edgeId, int baseNode, int adjNode, double cost, double score) {
        this.edgeId = edgeId;
        this.baseNode = baseNode;
        this.adjNode = adjNode;
        this.cost = cost;
        this.score = score;
    }

    @Override
    public String toString() {
        return "Arc{" +
                "edgeId=" + edgeId +
                '}';
    }

    @Override
    public boolean equals(Object o) {
        if(this == o) return true;
        if(o == null || getClass() != o.getClass()) return false;
        Arc arc = (Arc) o;
        return edgeId == arc.edgeId &&
                baseNode == arc.baseNode &&
                adjNode == arc.adjNode;
    }

    @Override
    public int hashCode() {

        return Objects.hash(edgeId, baseNode, adjNode);
    }
}

