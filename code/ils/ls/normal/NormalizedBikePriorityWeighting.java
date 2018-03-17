package com.graphhopper.routing.ils.ls.normal;

import com.graphhopper.routing.ils.BikePriorityWeighting;
import com.graphhopper.routing.ils.ls.normal.LSIteratedLocalSearch;
import com.graphhopper.routing.util.FlagEncoder;
import com.graphhopper.util.EdgeIteratorState;

/**
 * Weighting classed use to calculate scores of roads for the {@link LSIteratedLocalSearch} routing algorithm.
 */
class NormalizedBikePriorityWeighting extends BikePriorityWeighting {

    private double cutoff;

    NormalizedBikePriorityWeighting(FlagEncoder encoder, double cutoff) {
        super(encoder);
        this.cutoff = cutoff;
    }

    @Override
    public double calcWeight(EdgeIteratorState edgeState, boolean reverse, int prevOrNextEdgeId) {
        double weight = super.calcWeight(edgeState, reverse, prevOrNextEdgeId);
        return weight > cutoff ? 1 : 0;
    }
}
