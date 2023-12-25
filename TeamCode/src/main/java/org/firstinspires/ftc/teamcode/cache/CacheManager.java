package org.firstinspires.ftc.teamcode.cache;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class CacheManager {
    private final List<LynxModule> allHubs;

    public CacheManager(HardwareMap hardwareMap) {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void clear() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}
