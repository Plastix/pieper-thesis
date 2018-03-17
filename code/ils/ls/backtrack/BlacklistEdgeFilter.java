package com.graphhopper.routing.ils.ls.backtrack;

import com.carrotsearch.hppc.IntHashSet;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.util.EdgeIteratorState;
import com.sun.istack.internal.NotNull;

/**
 * EdgeFilter which blacklists a set EdgeIds. If an edge is not in the blacklist it defaults to the passed
 * in EdgeFilter.
 */
public class BlacklistEdgeFilter implements EdgeFilter {

    private EdgeFilter edgeFilter;
    private IntHashSet blacklist;

    BlacklistEdgeFilter(@NotNull EdgeFilter edgeFilter, @NotNull IntHashSet blacklist) {
        this.edgeFilter = edgeFilter;
        this.blacklist = blacklist;
    }

    @Override
    public boolean accept(EdgeIteratorState edgeState) {
        return !blacklist.contains(edgeState.getEdge()) && edgeFilter.accept(edgeState);
    }
}
