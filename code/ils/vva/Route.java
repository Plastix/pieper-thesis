package com.graphhopper.routing.ils.vva;

import com.carrotsearch.hppc.IntHashSet;
import com.graphhopper.routing.ils.IlsPath;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;

import java.util.ArrayList;
import java.util.List;

/**
 * Object which represents a path created by the {@link VVAIteratedLocalSearch} algorithm.
 */
final class Route {
    private List<Arc> arcs;
    private IntHashSet edges;
    private double cost;
    private double score;

    Route() {
        arcs = new ArrayList<>();
        edges = new IntHashSet();
    }

    // Copy constructor
    private Route(Route route) {
        cost = route.cost;
        score = route.score;
        arcs = new ArrayList<>(route.arcs);
        edges = route.edges.clone();
    }


    void addEdge(int edgeId, int baseNode, int adjNode, double cost, double score) {
        arcs.add(new Arc(edgeId, baseNode, adjNode, cost, score));
        edges.add(edgeId);
        this.cost += cost;
        this.score += score;
    }

    void removeEdge(int edgeId) {
        for(int i = arcs.size() - 1; i >= 0; i--) {
            Arc arc = arcs.get(i);
            if(arc.edgeId == edgeId) {
                arcs.remove(i);
                edges.remove(edgeId);
                cost -= arc.cost;
                score -= arc.score;
                break;
            }
        }
    }

    Arc removeEdgeIndex(int index) {
        Arc arc = arcs.remove(index);
        edges.remove(arc.edgeId);
        cost -= arc.cost;
        score -= arc.score;
        return arc;
    }

    void clear() {
        arcs.clear();
        edges.clear();
        cost = 0;
        score = 0;
    }

    Route copy() {
        return new Route(this);
    }

    boolean containsEdge(int edgeId) {
        return edges.contains(edgeId);
    }

    void insertRoute(Route other, int index) {
        arcs.addAll(index, other.arcs);
        edges.addAll(other.edges);
        cost += other.cost;
        score += other.score;
    }

    void blacklist(Route other) {
        edges.addAll(other.edges);
    }

    IlsPath getPath(Graph graph, Weighting costWeighting, Weighting scoreWeighting, int s, int d) {
        IlsPath path = new IlsPath(graph, costWeighting, scoreWeighting);
        for(Arc arc : arcs) {
            path.processEdge(arc.edgeId, arc.adjNode, arc.edgeId);
        }
        return (IlsPath) path
                .setEndNode(d)
                .setFromNode(s)
                .setFound(!arcs.isEmpty());
    }

    public double getCost() {
        return cost;
    }

    public double getScore() {
        return score;
    }

    public int length() {
        return arcs.size();
    }

}
