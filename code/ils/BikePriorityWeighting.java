package com.graphhopper.routing.ils;

import com.graphhopper.routing.util.FlagEncoder;
import com.graphhopper.routing.util.PriorityCode;
import com.graphhopper.routing.weighting.AbstractWeighting;
import com.graphhopper.util.EdgeIteratorState;

import static com.graphhopper.routing.weighting.PriorityWeighting.KEY;

/**
 * Weighting classed use to calculate scores of roads for the ILS routing algorithms.
 */
public class BikePriorityWeighting extends AbstractWeighting {

    public BikePriorityWeighting(FlagEncoder encoder) {
        super(encoder);
    }

    @Override
    public double getMinWeight(double distance) {
        return PriorityCode.WORST.getValue();
    }

    /**
     * Weight is a number which corresponds to the "goodness" of a road for a bike. This is set in
     * {@link com.graphhopper.routing.util.BikeCommonFlagEncoder}. Also see
     * https://wiki.openstreetmap.org/wiki/Key:class:bicycle
     */
    @Override
    public double calcWeight(EdgeIteratorState edgeState, boolean reverse, int prevOrNextEdgeId) {
        return flagEncoder.getDouble(edgeState.getFlags(), KEY);
    }

    @Override
    public String getName() {
        return "bike_priority";
    }
}
