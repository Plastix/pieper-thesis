package com.graphhopper.routing.ils.ls;

import com.graphhopper.routing.ils.ls.normal.LSIteratedLocalSearch;
import com.graphhopper.util.DistanceCalc;
import com.graphhopper.util.Helper;
import com.graphhopper.util.shapes.BBox;
import com.graphhopper.util.shapes.GHPoint;
import com.graphhopper.util.shapes.Shape;
import sun.reflect.generics.reflectiveObjects.NotImplementedException;

/**
 * Class which represents an Ellipse on the map. Used by the {@link LSIteratedLocalSearch} algorithm for restricting
 * the search space.
 * <p>
 * Note: This does not fully implement the Shape interface!
 */
public class Ellipse implements Shape {

    private static DistanceCalc calc = Helper.DIST_EARTH;

    private GHPoint focus1;
    private GHPoint focus2;
    private double radius;

    public Ellipse(GHPoint focus1, GHPoint focus2, double radius) {
        this.focus1 = focus1;
        this.focus2 = focus2;
        this.radius = radius;
    }

    @Override
    public boolean intersect(Shape o) {
        throw new NotImplementedException();
    }

    @Override
    public boolean contains(double lat, double lon) {
        return calc.calcDist(lat, lon, focus1.lat, focus1.lon) +
                calc.calcDist(lat, lon, focus2.lat, focus2.lon) <= radius;
    }

    @Override
    public boolean contains(Shape s) {
        throw new NotImplementedException();
    }

    @Override
    public BBox getBounds() {
        throw new NotImplementedException();
    }

    @Override
    public GHPoint getCenter() {
        throw new NotImplementedException();
    }


    @Override
    public double calculateArea() {
        throw new NotImplementedException();
    }
}
