package com.graphhopper.routing.ils;

import com.graphhopper.routing.RoutingAlgorithm;

public interface IlsAlgorithm extends RoutingAlgorithm {

    Iteration[] getIterationInfo();
}
