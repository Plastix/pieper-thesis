package com.graphhopper.routing.ils;

public class Iteration {
    private final double score;
    private final double time;

    public Iteration(double score, double time) {
        this.score = score;
        this.time = time;
    }

    public double getScore() {
        return score;
    }

    public double getTime() {
        return time;
    }
}
