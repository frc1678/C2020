package com.team1323.lib.geometry;

import com.team1323.lib.util.Interpolable;
import com.team254.lib.util.CSVWritable;

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}
