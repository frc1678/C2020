package com.team1678.frc2020.planners;

import com.team1678.frc2020.Constants;
import com.team1678.frc2020.subsystems.Indexer.ProxyStatus;
import com.team1678.frc2020.subsystems.Indexer.SlotStatus;


public class IndexerMotionPlanner {
    public IndexerMotionPlanner() {}

    protected double WrapDegrees(double degrees) {
        degrees = degrees % 360.0;
        degrees = (degrees + 360.0) % 360.0;
        if (degrees > 180.0)
            degrees -= 360.0;
        return degrees;
    }

    public int findNearestSlot(double indexer_angle, double turret_angle) {
        double wrappedIndexerAngle = WrapDegrees(indexer_angle);
        double wrappedTurretAngle = WrapDegrees(turret_angle);

        double offset = 0; 
        
        if (Math.abs(wrappedTurretAngle - wrappedIndexerAngle) > 180.0) {
            if (wrappedIndexerAngle > 0.0 && wrappedTurretAngle < 0.0) {
                offset = 360 - wrappedIndexerAngle + wrappedTurretAngle;
            } else if (wrappedIndexerAngle < 0.0 && wrappedTurretAngle > 0.0) {
                offset = -360 - wrappedIndexerAngle + wrappedTurretAngle;
            }
        } else {
            offset = wrappedTurretAngle - wrappedIndexerAngle;
        }

        int slotNumber = (int) Math.round(offset / Constants.kAnglePerSlot);

        return wrapSlotNumber(slotNumber);
    }

    public int findNearestSlotToIntake(double indexer_angle) {
        double wrappedIndexerAngle = WrapDegrees(indexer_angle);

        if (wrappedIndexerAngle < 0) {
            wrappedIndexerAngle += 360;
        }

        int slotNumber = (int) Math.round((wrappedIndexerAngle) / Constants.kAnglePerSlot);

        return wrapSlotNumber(slotNumber);
    }

    public int wrapSlotNumber(int slotNumber) {
        if (slotNumber < 0) {
            slotNumber += 5;
        } else if (slotNumber > 4) {
            slotNumber -= 5;
        }

        return slotNumber;
    }

    public SlotStatus updateSlotStatus(double indexer_angle, ProxyStatus proxy_status) {

        SlotStatus slotStatus = new SlotStatus();
        int frontSlot = findNearestSlotToIntake(indexer_angle);

        if (frontSlot == 0) {
            slotStatus.slot_zero = proxy_status.front_proxy;
            slotStatus.slot_one = proxy_status.right_proxy;
            slotStatus.slot_two = proxy_status.back_right_proxy;
            slotStatus.slot_three = proxy_status.back_left_proxy;
            slotStatus.slot_four = proxy_status.left_proxy;
        } else if (frontSlot == 1) {
            slotStatus.slot_one = proxy_status.front_proxy;
            slotStatus.slot_two = proxy_status.right_proxy;
            slotStatus.slot_three = proxy_status.back_right_proxy;
            slotStatus.slot_four = proxy_status.back_left_proxy;
            slotStatus.slot_zero = proxy_status.left_proxy;
        } else if (frontSlot == 2) {
            slotStatus.slot_two = proxy_status.front_proxy;
            slotStatus.slot_three = proxy_status.right_proxy;
            slotStatus.slot_four = proxy_status.back_right_proxy;
            slotStatus.slot_zero = proxy_status.back_left_proxy;
            slotStatus.slot_one = proxy_status.left_proxy;
        } else if (frontSlot == 3) {
            slotStatus.slot_three = proxy_status.front_proxy;
            slotStatus.slot_four = proxy_status.right_proxy;
            slotStatus.slot_zero = proxy_status.back_right_proxy;
            slotStatus.slot_one = proxy_status.back_left_proxy;
            slotStatus.slot_two = proxy_status.left_proxy;
        } else if (frontSlot == 4) {
            slotStatus.slot_four = proxy_status.front_proxy;
            slotStatus.slot_zero = proxy_status.right_proxy;
            slotStatus.slot_one = proxy_status.back_right_proxy;
            slotStatus.slot_two = proxy_status.back_left_proxy;
            slotStatus.slot_three = proxy_status.left_proxy;
        }

        return slotStatus;
    }

    public double findAngleToIntake(int slotNumber, double indexer_angle) {
        double wrappedIndexerAngle = WrapDegrees(indexer_angle);

        double slotAngle = WrapDegrees(slotNumber * Constants.kAnglePerSlot) - wrappedIndexerAngle;

        double angleGoal = WrapDegrees(slotAngle);

        return angleGoal;
    }

    public double findAngleGoalToIntake(int slotNumber, double indexer_angle) {
        return findAngleToIntake(slotNumber, indexer_angle);
    }

    public double findSnappedAngleToGoal(double indexer_angle) {
        double wrappedIndexerAngle = WrapDegrees(indexer_angle);
        double angleGoal;

        if (wrappedIndexerAngle >= 0) {
            angleGoal = 72 - wrappedIndexerAngle % 72;
        } else {
            angleGoal = -72 - wrappedIndexerAngle % 72;
        }

        if (Math.abs(angleGoal) > 36) {
            angleGoal -= 72 * Math.signum(angleGoal);
        }

        return angleGoal;
    }

    public double findSnappedAngleGoal(double indexer_angle) {
        return findSnappedAngleToGoal(indexer_angle) + indexer_angle;
    }

    public boolean isSnapped(double indexer_angle) {
        return Math.abs(indexer_angle % 72) < Constants.kIndexerDeadband;
    }

    public int findNextSlot(double indexer_angle, double turret_angle) {
        int currentSlot = findNearestSlot(indexer_angle, turret_angle);

        return wrapSlotNumber(currentSlot + 1);
    }

    public int findPreviousSlot(double indexer_angle, double turret_angle) {
        int currentSlot = findNearestSlot(indexer_angle, turret_angle);

        return wrapSlotNumber(currentSlot - 1);    
    }

    public double findAngleGoal(int slotNumber, double indexer_angle, double turret_angle) {
        return findAngleToGoal(slotNumber, indexer_angle, turret_angle) + indexer_angle;
    }
    
    public double findAngleToGoal(int slotNumber, double indexer_angle, double turret_angle) {
        double wrappedIndexerAngle = WrapDegrees(indexer_angle);
        double wrappedTurretAngle = WrapDegrees(turret_angle);

        double slotAngle = WrapDegrees(slotNumber * Constants.kAnglePerSlot) + wrappedIndexerAngle;

        double angleGoal = WrapDegrees(wrappedTurretAngle - slotAngle);

        return angleGoal;
    }

    public double findNearestDeadSpot(double indexer_angle, double turret_angle) {
        double wrappedIndexerAngle = WrapDegrees(indexer_angle);
        double wrappedTurretAngle = WrapDegrees(turret_angle);

        int slotNumber = findNearestSlot(indexer_angle, turret_angle);
        double slotAngle = WrapDegrees(slotNumber * Constants.kAnglePerSlot) + wrappedIndexerAngle;

        double angleGoal = WrapDegrees(wrappedTurretAngle - slotAngle);

        if (angleGoal >= 0) {
            angleGoal -= 36;
        } else {
            angleGoal += 36;
        }

        return angleGoal;
    }

    public boolean isAtGoal(int slotNumber, double indexer_angle, double turret_angle) {
        return Math.abs(findAngleToGoal(slotNumber, indexer_angle, turret_angle)) < Constants.kIndexerDeadband;
    }

    public boolean isAtDeadSpot(double indexer_angle, double turret_angle) {
        return Math.abs(findNearestDeadSpot(indexer_angle, turret_angle)) < Constants.kIndexerDeadband;
    }

    public int findNearestOpenSlot(double indexer_angle, ProxyStatus proxy_status) {
        int currentSlot = findNearestSlotToIntake(indexer_angle);
        int slotGoal;

        boolean frontProxy = proxy_status.front_proxy;
        boolean rightProxy = proxy_status.right_proxy;
        boolean leftProxy = proxy_status.left_proxy;
        boolean backRightProxy = proxy_status.back_right_proxy;
        boolean backLeftProxy = proxy_status.back_left_proxy;
        
        if (!frontProxy) {
            slotGoal = currentSlot;
        } else {
            if (!rightProxy) {
                slotGoal = currentSlot + 1;
            } else {
                if (!leftProxy) {
                    slotGoal = currentSlot - 1;
                } else {
                    if (!backRightProxy) {
                        slotGoal = currentSlot + 2;
                    } else {
                        if (!backLeftProxy) {
                            slotGoal = currentSlot - 2;
                        } else {
                            slotGoal = currentSlot; // all slots filled
                        }
                    }
                }
            }
        }

        return wrapSlotNumber(slotGoal);
    }
}
