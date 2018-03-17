package com.graphhopper.routing.ils.ls.backtrack;

import com.graphhopper.routing.DijkstraBidirectionRef;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.util.TraversalMode;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;

public class IlsDijkstraSearch extends DijkstraBidirectionRef {
    IlsDijkstraSearch(Graph graph, Weighting weighting, TraversalMode traversalMode) {
        super(graph, weighting, traversalMode);
    }

    @Override
    protected Path createAndInitPath() {
        bestPath = new IlsPathCh(graph, weighting);
        return bestPath;
    }
}
