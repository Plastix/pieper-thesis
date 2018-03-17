package com.graphhopper.routing.ils.ls.normal;

import com.graphhopper.routing.Path;
import com.graphhopper.routing.ils.ls.Arc;
import com.sun.istack.internal.NotNull;

public interface ShortestPathCalculator {

    /**
     * Returns the shortest distance in meters between two nodes of the graph.
     */
    Path shortestPath(int s, int d);

    /**
     * Returns the total distance in meters of the path s --> arc --> d where "-->" is shortest path.
     *
     * @param s   Start node ID.
     * @param d   End node ID.
     * @param arc Arc.
     * @return Distance in meters
     */
    double getPathCost(int s, int d, @NotNull Arc arc);

}
