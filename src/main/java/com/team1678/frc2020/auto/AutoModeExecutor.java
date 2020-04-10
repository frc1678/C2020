package com.team1678.frc2020.auto;

import com.team1678.frc2020.auto.modes.AutoModeBase;
import com.team254.lib.util.CrashTrackingRunnable;

/**
 * This class selects, runs, and (if necessary) stops a specified autonomous mode.
 */
public class AutoModeExecutor {
    private static AutoModeExecutor mInstance = null;

    private AutoModeBase mAutoMode = null;
    private Thread mThread = null;

    public AutoModeExecutor() {}

    public static AutoModeExecutor getInstance() {
        if (mInstance == null) {
            mInstance = new AutoModeExecutor();
        }

        return mInstance;
    }

    public void setAutoMode(AutoModeBase new_auto_mode) {
        mAutoMode = new_auto_mode;
        mThread = new Thread(new CrashTrackingRunnable() {
            @Override
            public void runCrashTracked() {
                if (mAutoMode != null) {
                    mAutoMode.run();
                }
            }
        });
    }

    public void start() {
        if (mThread != null) {
            mThread.start();
        }
    }

    public boolean isStarted() {
        return mAutoMode != null && mAutoMode.isActive() && mThread != null && mThread.isAlive();
    }

    public void reset() {
        if (isStarted()) {
            stop();
        }

        mAutoMode = null;
    }

    public void stop() {
        if (mAutoMode != null) {
            mAutoMode.stop();
        }

        mThread = null;
    }

    public AutoModeBase getAutoMode() {
        return mAutoMode;
    }

}