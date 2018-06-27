package com.graphhopper.routing.ils.ls.normal;

import com.graphhopper.routing.ils.BikePriorityWeighting;
import com.graphhopper.routing.util.FlagEncoder;
import com.graphhopper.util.EdgeIteratorState;

public class ScaledBikePriorityWeighting extends BikePriorityWeighting {
    public ScaledBikePriorityWeighting(FlagEncoder encoder) {
        super(encoder);
    }

    @Override
    public double calcWeight(EdgeIteratorState edgeState, boolean reverse, int prevOrNextEdgeId) {
        return edgeState.getDistance() * super.calcWeight(edgeState, reverse, prevOrNextEdgeId);
    }
}
