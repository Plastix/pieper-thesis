/*
 *  Licensed to GraphHopper GmbH under one or more contributor
 *  license agreements. See the NOTICE file distributed with this work for
 *  additional information regarding copyright ownership.
 *
 *  GraphHopper GmbH licenses this file to you under the Apache License,
 *  Version 2.0 (the "License"); you may not use this file except in
 *  compliance with the License. You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
package com.graphhopper.routing.ils;

import com.graphhopper.GHRequest;
import com.graphhopper.GHResponse;
import com.graphhopper.PathWrapper;
import com.graphhopper.routing.AlgorithmOptions;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.QueryGraph;
import com.graphhopper.routing.RoutingAlgorithmFactory;
import com.graphhopper.routing.template.AbstractRoutingTemplate;
import com.graphhopper.routing.template.RoutingTemplate;
import com.graphhopper.routing.util.DefaultEdgeFilter;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.util.FlagEncoder;
import com.graphhopper.routing.util.NameSimilarityEdgeFilter;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.storage.index.QueryResult;
import com.graphhopper.util.PMap;
import com.graphhopper.util.PathMerger;
import com.graphhopper.util.Translation;
import com.graphhopper.util.exceptions.PointNotFoundException;
import com.graphhopper.util.shapes.GHPoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static com.graphhopper.util.Parameters.Routing.*;


@SuppressWarnings("Duplicates")
public class TestRunnerRoutingTemplate extends AbstractRoutingTemplate implements RoutingTemplate {
    private final Logger logger = LoggerFactory.getLogger(getClass());

    protected final GHRequest ghRequest;
    protected final GHResponse ghResponse;
    protected final PathWrapper altResponse = new PathWrapper();
    private final LocationIndex locationIndex;

    public TestRunnerRoutingTemplate(GHRequest ghRequest, GHResponse ghRsp, LocationIndex locationIndex) {
        this.locationIndex = locationIndex;
        this.ghRequest = ghRequest;
        this.ghResponse = ghRsp;
    }

    @Override
    public List<QueryResult> lookup(List<GHPoint> points, FlagEncoder encoder) {
        if(points.size() < 2)
            throw new IllegalArgumentException("At least 2 points have to be specified, but was:" + points.size());

        EdgeFilter edgeFilter = new DefaultEdgeFilter(encoder);
        queryResults = new ArrayList<>(points.size());
        for(int placeIndex = 0; placeIndex < points.size(); placeIndex++) {
            GHPoint point = points.get(placeIndex);
            QueryResult res;
            if(ghRequest.hasPointHints()) {
                res = locationIndex.findClosest(point.lat, point.lon, new NameSimilarityEdgeFilter(edgeFilter, ghRequest.getPointHints().get(placeIndex)));
                if(!res.isValid()) {
                    res = locationIndex.findClosest(point.lat, point.lon, edgeFilter);
                }
            } else {
                res = locationIndex.findClosest(point.lat, point.lon, edgeFilter);
            }
            if(!res.isValid())
                ghResponse.addError(new PointNotFoundException("Cannot find point " + placeIndex + ": " + point, placeIndex));

            queryResults.add(res);
        }

        return queryResults;
    }

    @Override
    public List<Path> calcPaths(QueryGraph queryGraph, RoutingAlgorithmFactory algoFactory, AlgorithmOptions algoOpts) {

        PMap hints = algoOpts.getHints();
        String outputFileName = hints.get(OUTPUT_FILE, DEFAULT_OUTPUT_FILE);

        try {
            File file = new File(outputFileName);
            if(file.exists()) {
                file.delete();
            }
            file.createNewFile();

            FileWriter writer = new FileWriter(file, true);
            StringBuilder builder = new StringBuilder();
            builder.append("iteration,score,time\n");

            QueryResult fromQResult = queryResults.get(0);
            // enforce start direction
            queryGraph.enforceHeading(fromQResult.getClosestNode(), ghRequest.getFavoredHeading(0), false);
            QueryResult toQResult = queryResults.get(1);
            // enforce end direction
            queryGraph.enforceHeading(toQResult.getClosestNode(), ghRequest.getFavoredHeading(1), true);
            int start = fromQResult.getClosestNode();
            int end = toQResult.getClosestNode();

            int runs = hints.getInt(NUM_RUNS, DEFAULT_NUM_RUNS);
            for(int i = 1; i <= runs; i++) {
                try {
                    hints.put(SEED, System.currentTimeMillis());
                    IlsAlgorithm ils = (IlsAlgorithm) algoFactory.createAlgo(queryGraph, algoOpts);
                    ils.calcPath(start, end);

                    Iteration[] iterationInfo = ils.getIterationInfo();
                    for(int j = 0; j < iterationInfo.length; j++) {
                        Iteration iteration = iterationInfo[j];
                        builder.append(String.format("%d,%f,%f\n", j + 1, iteration.getScore(), iteration.getTime()));
                    }

                    if(i % 10 == 0) {
                        logger.info("{} percent complete!", String.format("%.2f", ((double) i / runs) * 100.0));
                    }

                } catch(ClassCastException ex) {
                    logger.error("You can only use this routing template with ILS algorithms!");
                    break;
                }
            }

            writer.append(builder);
            writer.close();

            // reset all direction enforcements in queryGraph to avoid influencing next path
            queryGraph.clearUnfavoredStatus();

        } catch(IOException e) {
            e.printStackTrace();
        }

        // Don't return anything
        // We're just recording data
        return null;
    }

    @Override
    public boolean isReady(PathMerger pathMerger, Translation tr) {
        altResponse.setWaypoints(getWaypoints());
        ghResponse.add(altResponse);
        return false;
    }

    @Override
    public int getMaxRetries() {
        return 1;
    }
}
