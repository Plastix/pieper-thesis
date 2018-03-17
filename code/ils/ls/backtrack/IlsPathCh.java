package com.graphhopper.routing.ils.ls.backtrack;

import com.carrotsearch.hppc.IntHashSet;
import com.graphhopper.routing.PathBidirRef;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;

public class IlsPathCh extends PathBidirRef {

    private IntHashSet edges;

    IlsPathCh(Graph g, Weighting weighting) {
        super(g, weighting);
        this.edges = new IntHashSet();
    }

    @Override
    protected void addEdge(int edge) {
        super.addEdge(edge);
        edges.add(edge);
    }

    public IntHashSet getEdges() {
        return edges;
    }
}
