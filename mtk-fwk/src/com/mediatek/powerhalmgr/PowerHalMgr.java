package com.mediatek.powerhalmgr;

public class PowerHalMgr {

    // Display Frame Rate Modes
    public static final int DFPS_MODE_ARR = 0x2;
    public static final int DFPS_MODE_DEFAULT = 0x0;
    public static final int DFPS_MODE_FRR = 0x1;
    public static final int DFPS_MODE_INTERNAL_SW = 0x3;
    public static final int DFPS_MODE_MAXIMUM = 0x4;

    // Display Modes
    public static final int MTKPOWER_DISP_MODE_DEFAULT = 0x0;
    public static final int MTKPOWER_DISP_MODE_EN = 0x1;
    public static final int MTKPOWER_DISP_MODE_NUM = 0x2;

    // Power Hint Constants
    public static final int MTKPOWER_HINT_ALWAYS_ENABLE = 0xfffffff;

    // Screen Off States
    public static final int MTKPOWER_SCREEN_OFF_DISABLE = 0x0;
    public static final int MTKPOWER_SCREEN_OFF_ENABLE = 0x1;
    public static final int MTKPOWER_SCREEN_OFF_WAIT_RESTORE = 0x2;

    // Power States
    public static final int MTKPOWER_STATE_DEAD = 0x3;
    public static final int MTKPOWER_STATE_DESTORYED = 0x2;
    public static final int MTKPOWER_STATE_PAUSED = 0x0;
    public static final int MTKPOWER_STATE_RESUMED = 0x1;
    public static final int MTKPOWER_STATE_STOPPED = 0x4;

    // AI Resources
    public static final int PERF_RES_AI_APUSYS_BOOST_IPU_IF = 0x1810000;
    public static final int PERF_RES_AI_MDLA_FREQ_MAX = 0x180c000;
    public static final int PERF_RES_AI_MDLA_FREQ_MIN = 0x1808000;
    public static final int PERF_RES_AI_VPU_FREQ_MAX_CORE_0 = 0x1804000;
    public static final int PERF_RES_AI_VPU_FREQ_MAX_CORE_1 = 0x1804100;
    public static final int PERF_RES_AI_VPU_FREQ_MIN_CORE_0 = 0x1800000;
    public static final int PERF_RES_AI_VPU_FREQ_MIN_CORE_1 = 0x1800100;

    // CFP (Critical Frame Protection) Resources
    public static final int PERF_RES_CFP_DOWN_LOADING = 0x3008300;
    public static final int PERF_RES_CFP_DOWN_OPP = 0x3008700;
    public static final int PERF_RES_CFP_DOWN_TIME = 0x3008500;
    public static final int PERF_RES_CFP_ENABLE = 0x3008000;
    public static final int PERF_RES_CFP_POLLING_MS = 0x3008100;
    public static final int PERF_RES_CFP_UP_LOADING = 0x3008200;
    public static final int PERF_RES_CFP_UP_OPP = 0x3008600;
    public static final int PERF_RES_CFP_UP_TIME = 0x3008400;

    // CPU Core Resources
    public static final int PERF_RES_CPUCORE_MAX_CLUSTER_0 = 0x804000;
    public static final int PERF_RES_CPUCORE_MAX_CLUSTER_1 = 0x804100;
    public static final int PERF_RES_CPUCORE_MIN_CLUSTER_0 = 0x800000;
    public static final int PERF_RES_CPUCORE_MIN_CLUSTER_1 = 0x800100;

    // CPU Frequency Resources
    public static final int PERF_RES_CPUFREQ_CCI_FREQ = 0x410000;
    public static final int PERF_RES_CPUFREQ_MAX_CLUSTER_0 = 0x404000;
    public static final int PERF_RES_CPUFREQ_MAX_CLUSTER_1 = 0x404100;
    public static final int PERF_RES_CPUFREQ_MAX_HL_CLUSTER_0 = 0x40c000;
    public static final int PERF_RES_CPUFREQ_MAX_HL_CLUSTER_1 = 0x40c100;
    public static final int PERF_RES_CPUFREQ_MIN_CLUSTER_0 = 0x400000;
    public static final int PERF_RES_CPUFREQ_MIN_CLUSTER_1 = 0x400100;
    public static final int PERF_RES_CPUFREQ_MIN_HL_CLUSTER_0 = 0x408000;
    public static final int PERF_RES_CPUFREQ_MIN_HL_CLUSTER_1 = 0x408100;
    public static final int PERF_RES_CPUFREQ_PERF_MODE = 0x414000;

    // Custom Resources
    public static final int PERF_RES_CUSTOM_RESOURCE_1 = 0x10000000;

    // Display Resources
    public static final int PERF_RES_DISP_DECOUPLE = 0x2408000;
    public static final int PERF_RES_DISP_DFPS_FPS = 0x2400100;
    public static final int PERF_RES_DISP_DFPS_MODE = 0x2400000;
    public static final int PERF_RES_DISP_IDLE_TIME = 0x240c000;
    public static final int PERF_RES_DISP_VIDEO_MODE = 0x2404000;

    // DRAM Resources
    public static final int PERF_RES_DRAM_CM_MGR = 0x1010000;
    public static final int PERF_RES_DRAM_CM_MGR_CAM_ENABLE = 0x1010100;
    public static final int PERF_RES_DRAM_CM_RATIO_UP_X_0 = 0x1014000;
    public static final int PERF_RES_DRAM_CM_RATIO_UP_X_1 = 0x1014100;
    public static final int PERF_RES_DRAM_CM_RATIO_UP_X_2 = 0x1014200;
    public static final int PERF_RES_DRAM_CM_RATIO_UP_X_3 = 0x1014300;
    public static final int PERF_RES_DRAM_CM_RATIO_UP_X_4 = 0x1014400;
    public static final int PERF_RES_DRAM_OPP_MIN = 0x1000000;
    public static final int PERF_RES_DRAM_VCORE_BW_ENABLE = 0x1008000;
    public static final int PERF_RES_DRAM_VCORE_BW_THRES = 0x1008100;
    public static final int PERF_RES_DRAM_VCORE_BW_THRESH_LP3 = 0x1008200;
    public static final int PERF_RES_DRAM_VCORE_MIN = 0x1004000;
    public static final int PERF_RES_DRAM_VCORE_MIN_LP3 = 0x1004100;
    public static final int PERF_RES_DRAM_VCORE_POLICY = 0x100c000;

    // FPS Resources
    public static final int PERF_RES_FPS_EARA_BENCH = 0x2028000;
    public static final int PERF_RES_FPS_EARA_THERMAL_ENABLE = 0x2038000;
    public static final int PERF_RES_FPS_FBT_BHR = 0x2024000;
    public static final int PERF_RES_FPS_FBT_BHR_OPP = 0x2020000;
    public static final int PERF_RES_FPS_FBT_BOOST_TA = 0x2030000;
    public static final int PERF_RES_FPS_FBT_DEQTIME_BOUND = 0x2010000;
    public static final int PERF_RES_FPS_FBT_FLOOR_BOUND = 0x2014000;
    public static final int PERF_RES_FPS_FBT_KMIN = 0x2018000;
    public static final int PERF_RES_FPS_FBT_MIN_RESCUE_PERCENT = 0x200c100;
    public static final int PERF_RES_FPS_FBT_RESCUE_C = 0x203c300;
    public static final int PERF_RES_FPS_FBT_RESCUE_F = 0x203c000;
    public static final int PERF_RES_FPS_FBT_RESCUE_PERCENT = 0x203c100;
    public static final int PERF_RES_FPS_FBT_SHORT_RESCUE_NS = 0x200c000;
    public static final int PERF_RES_FPS_FBT_ULTRA_RESCUE = 0x203c200;
    public static final int PERF_RES_FPS_FPSGO_ADJ_CNT = 0x2040100;
    public static final int PERF_RES_FPS_FPSGO_ADJ_DEBNC_CNT = 0x2040200;
    public static final int PERF_RES_FPS_FPSGO_ADJ_LOADING = 0x2040000;
    public static final int PERF_RES_FPS_FPSGO_ADJ_LOADING_TIMEDIFF = 0x2040300;
    public static final int PERF_RES_FPS_FPSGO_DEP_FRAMES = 0x2048000;
    public static final int PERF_RES_FPS_FPSGO_ENABLE = 0x2004000;
    public static final int PERF_RES_FPS_FPSGO_GPU_BLOCK_BOOST = 0x202c000;
    public static final int PERF_RES_FPS_FPSGO_IDLEPREFER = 0x2050000;
    public static final int PERF_RES_FPS_FPSGO_LLF_POLICY = 0x2044100;
    public static final int PERF_RES_FPS_FPSGO_LLF_TH = 0x2044000;
    public static final int PERF_RES_FPS_FPSGO_MARGIN_MODE = 0x2034000;
    public static final int PERF_RES_FPS_FPSGO_MARGIN_MODE_DBNC_A = 0x2034100;
    public static final int PERF_RES_FPS_FPSGO_MARGIN_MODE_DBNC_B = 0x2034200;
    public static final int PERF_RES_FPS_FPSGO_SP_CK_PERIOD = 0x2048300;
    public static final int PERF_RES_FPS_FPSGO_SP_NAME_ID = 0x2048100;
    public static final int PERF_RES_FPS_FPSGO_SP_SUB = 0x2048200;
    public static final int PERF_RES_FPS_FSTB_FORCE_VAG = 0x2008000;
    public static final int PERF_RES_FPS_FSTB_FPS_LOWER = 0x2000000;
    public static final int PERF_RES_FPS_FSTB_FPS_UPPER = 0x2000100;
    public static final int PERF_RES_FPS_FSTB_SOFT_FPS_LOWER = 0x201c000;
    public static final int PERF_RES_FPS_FSTB_SOFT_FPS_UPPER = 0x201c100;
    public static final int PERF_RES_FPS_GBE1_ENABLE = 0x204c000;
    public static final int PERF_RES_FPS_GBE2_ENABLE = 0x204c100;
    public static final int PERF_RES_FPS_GBE2_LOADING_TH = 0x204c300;
    public static final int PERF_RES_FPS_GBE2_MAX_BOOST_CNT = 0x204c400;
    public static final int PERF_RES_FPS_GBE2_TIMER2_MS = 0x204c200;
    public static final int PERF_RES_FPS_GBE_POLICY_MASK = 0x204c500;

    // GPU Resources
    public static final int PERF_RES_GPU_FREQ_LOW_LATENCY = 0xc08000;
    public static final int PERF_RES_GPU_FREQ_MAX = 0xc04000;
    public static final int PERF_RES_GPU_FREQ_MIN = 0xc00000;
    public static final int PERF_RES_GPU_GED_CWAITG = 0xc0c400;
    public static final int PERF_RES_GPU_GED_DVFS_LOADING_MODE = 0xc0c600;
    public static final int PERF_RES_GPU_GED_GX_BOOST = 0xc0c500;
    public static final int PERF_RES_GPU_GED_LOADING_BASE_DVFS_STEP = 0xc0c300;
    public static final int PERF_RES_GPU_GED_MARGIN_MODE = 0xc0c100;
    public static final int PERF_RES_GPU_GED_TIMER_BASE_DVFS_MARGIN = 0xc0c200;
    public static final int PERF_RES_GPU_GED_UNUSED_1 = 0xc0c000;
    public static final int PERF_RES_GPU_POWER_POLICY = 0xc10000;

    // I/O Resources
    public static final int PERF_RES_IO_BLKDEV_READAHEAD = 0x2c0c000;
    public static final int PERF_RES_IO_BOOST_VALUE = 0x2c00000;
    public static final int PERF_RES_IO_DATA_FS_BOOST = 0x2c14000;
    public static final int PERF_RES_IO_EXT4_DATA_BOOST = 0x2c10000;
    public static final int PERF_RES_IO_F2FS_EMMC_BOOST = 0x2c08000;
    public static final int PERF_RES_IO_F2FS_UFS_BOOST = 0x2c04000;
    public static final int PERF_RES_IO_F2FS_UFS_BOOST_ULTRA = 0x2c04100;
    public static final int PERF_RES_IO_UCLAMP_MIN = 0x2c00100;

    // Network Resources
    public static final int PERF_RES_NET_BT_A2DP_LOW_LATENCY = 0x2810000;
    public static final int PERF_RES_NET_MD_CERT_PID = 0x280c200;
    public static final int PERF_RES_NET_MD_CRASH_PID = 0x280c300;
    public static final int PERF_RES_NET_MD_GAME_MODE = 0x280c100;
    public static final int PERF_RES_NET_MD_LOW_LATENCY = 0x280c000;
    public static final int PERF_RES_NET_MD_WEAK_SIG_OPT = 0x280c400;
    public static final int PERF_RES_NET_NETD_BLOCK_UID = 0x2808100;
    public static final int PERF_RES_NET_NETD_BOOST_UID = 0x2808000;
    public static final int PERF_RES_NET_WIFI_CAM = 0x2800000;
    public static final int PERF_RES_NET_WIFI_LOW_LATENCY = 0x2804000;
    public static final int PERF_RES_NET_WIFI_SMART_PREDICT = 0x2804100;

    // Performance Resources
    public static final int PERF_RES_PERF_TASK_TURBO = 0x300c000;

    // PowerHAL Resources
    public static final int PERF_RES_POWERHAL_SCN_CRASH = 0x3400100;
    public static final int PERF_RES_POWERHAL_SCREEN_OFF_STATE = 0x3400000;
    public static final int PERF_RES_POWERHAL_SPORTS_MODE = 0x3404000;
    public static final int PERF_RES_POWERHAL_SPORTS_MODE_APP_SMART_MODE = 0x3404100;
    public static final int PERF_RES_POWERHAL_TOUCH_BOOST_ACTIVE_TIME = 0x3408200;
    public static final int PERF_RES_POWERHAL_TOUCH_BOOST_DURATION = 0x3408100;
    public static final int PERF_RES_POWERHAL_TOUCH_BOOST_EAS_BOOST = 0x3408300;
    public static final int PERF_RES_POWERHAL_TOUCH_BOOST_ENABLE = 0x3408500;
    public static final int PERF_RES_POWERHAL_TOUCH_BOOST_NOTIFY_FBC = 0x3408600;
    public static final int PERF_RES_POWERHAL_TOUCH_BOOST_OPP = 0x3408000;
    public static final int PERF_RES_POWERHAL_TOUCH_BOOST_TIME_TO_LAST_TOUCH = 0x3408400;
    public static final int PERF_RES_POWERHAL_WHITELIST_ACT_SWITCH_TIME = 0x340c200;
    public static final int PERF_RES_POWERHAL_WHITELIST_APP_LAUNCH_TIME_COLD = 0x340c000;
    public static final int PERF_RES_POWERHAL_WHITELIST_APP_LAUNCH_TIME_WARM = 0x340c100;

    // Power Resources
    public static final int PERF_RES_POWER_CPUFREQ_ABOVE_HISPEED_DELAY = 0x1c08000;
    public static final int PERF_RES_POWER_CPUFREQ_HISPEED_FREQ = 0x1c00000;
    public static final int PERF_RES_POWER_CPUFREQ_MIN_SAMPLE_TIME = 0x1c04000;
    public static final int PERF_RES_POWER_CPUFREQ_POWER_MODE = 0x1c0c000;
    public static final int PERF_RES_POWER_CPUIDLE_MCDI_ENABLE = 0x1c3c000;
    public static final int PERF_RES_POWER_HINT_EXT_HINT = 0x3410100;
    public static final int PERF_RES_POWER_HINT_EXT_HINT_FOR_GAME = 0x3410500;
    public static final int PERF_RES_POWER_HINT_EXT_HINT_HOLD_TIME = 0x3410200;
    public static final int PERF_RES_POWER_HINT_HOLD_TIME = 0x3410000;
    public static final int PERF_RES_POWER_HPS_HEAVY_TASK = 0x1c1c000;
    public static final int PERF_RES_POWER_HPS_POWER_MODE = 0x1c20000;
    public static final int PERF_RES_POWER_HPS_RUSH_BOOST_ENABLE = 0x1c18000;
    public static final int PERF_RES_POWER_HPS_RUSH_BOOST_THRESH = 0x1c18100;
    public static final int PERF_RES_POWER_HPS_THRESH_DOWN = 0x1c10100;
    public static final int PERF_RES_POWER_HPS_THRESH_UP = 0x1c10000;
    public static final int PERF_RES_POWER_HPS_TIMES_DOWN = 0x1c14100;
    public static final int PERF_RES_POWER_HPS_TIMES_UP = 0x1c14000;
    public static final int PERF_RES_POWER_PPM_HICA_VAR = 0x1c2c000;
    public static final int PERF_RES_POWER_PPM_LIMIT_BIG = 0x1c30000;
    public static final int PERF_RES_POWER_PPM_MODE = 0x1c28000;
    public static final int PERF_RES_POWER_PPM_ROOT_CLUSTER = 0x1c24000;
    public static final int PERF_RES_POWER_PPM_SPORTS_MODE = 0x1c34000;
    public static final int PERF_RES_POWER_PPM_USERLIMIT_BOOST = 0x1c38000;
    public static final int PERF_RES_POWER_SYSLIMITER_120 = 0x1c40200;
    public static final int PERF_RES_POWER_SYSLIMITER_60 = 0x1c40000;
    public static final int PERF_RES_POWER_SYSLIMITER_90 = 0x1c40100;
    public static final int PERF_RES_POWER_SYSLIMITER_DISABLE = 0x1c44000;

    // Scheduler Resources
    public static final int PERF_RES_SCHED_BOOST = 0x1410000;
    public static final int PERF_RES_SCHED_BOOST_VALUE_BG = 0x1400200;
    public static final int PERF_RES_SCHED_BOOST_VALUE_FG = 0x1400100;
    public static final int PERF_RES_SCHED_BOOST_VALUE_ROOT = 0x1400000;
    public static final int PERF_RES_SCHED_BOOST_VALUE_RT = 0x1400400;
    public static final int PERF_RES_SCHED_BOOST_VALUE_TA = 0x1400300;
    public static final int PERF_RES_SCHED_BTASK_ROTATE = 0x141c000;
    public static final int PERF_RES_SCHED_CACHE_AUDIT = 0x1420000;
    public static final int PERF_RES_SCHED_CPU_PREFER_TASK_1_BIG = 0x1418000;
    public static final int PERF_RES_SCHED_CPU_PREFER_TASK_1_LITTLE = 0x1418100;
    public static final int PERF_RES_SCHED_CPU_PREFER_TASK_1_RESERVED = 0x1418200;
    public static final int PERF_RES_SCHED_CPU_PREFER_TASK_2_BIG = 0x1418300;
    public static final int PERF_RES_SCHED_CPU_PREFER_TASK_2_LITTLE = 0x1418400;
    public static final int PERF_RES_SCHED_CPU_PREFER_TASK_2_RESERVED = 0x1418500;
    public static final int PERF_RES_SCHED_HEAVY_TASK_AVG_HTASK_AC = 0x142c100;
    public static final int PERF_RES_SCHED_HEAVY_TASK_AVG_HTASK_THRES = 0x142c200;
    public static final int PERF_RES_SCHED_HEAVY_TASK_THRES = 0x142c000;
    public static final int PERF_RES_SCHED_MIGRATE_COST = 0x1414000;
    public static final int PERF_RES_SCHED_MTK_PREFER_IDLE = 0x1428000;
    public static final int PERF_RES_SCHED_PLUS_DOWN_THROTTLE_NS = 0x1424000;
    public static final int PERF_RES_SCHED_PLUS_SYNC_FLAG = 0x1424200;
    public static final int PERF_RES_SCHED_PLUS_UP_THROTTLE_NS = 0x1424100;
    public static final int PERF_RES_SCHED_PREFER_CPU_BG = 0x1434200;
    public static final int PERF_RES_SCHED_PREFER_CPU_FG = 0x1434100;
    public static final int PERF_RES_SCHED_PREFER_CPU_ROOT = 0x1434000;
    public static final int PERF_RES_SCHED_PREFER_CPU_RT = 0x1434400;
    public static final int PERF_RES_SCHED_PREFER_CPU_TA = 0x1434300;
    public static final int PERF_RES_SCHED_PREFER_IDLE_BG = 0x1404200;
    public static final int PERF_RES_SCHED_PREFER_IDLE_FG = 0x1404100;
    public static final int PERF_RES_SCHED_PREFER_IDLE_ROOT = 0x1404000;
    public static final int PERF_RES_SCHED_PREFER_IDLE_RT = 0x1404400;
    public static final int PERF_RES_SCHED_PREFER_IDLE_TA = 0x1404300;
    public static final int PERF_RES_SCHED_TUNE_THRES = 0x140c000;
    public static final int PERF_RES_SCHED_UCLAMP_MIN_BG = 0x1408200;
    public static final int PERF_RES_SCHED_UCLAMP_MIN_FG = 0x1408100;
    public static final int PERF_RES_SCHED_UCLAMP_MIN_ROOT = 0x1408000;
    public static final int PERF_RES_SCHED_UCLAMP_MIN_RT = 0x1408400;
    public static final int PERF_RES_SCHED_UCLAMP_MIN_TA = 0x1408300;
    public static final int PERF_RES_SCHED_WALT = 0x1430000;

    // Thermal Resources
    public static final int PERF_RES_THERMAL_POLICY = 0x3000000;

    // Touch Resources
    public static final int PERF_RES_TOUCH_CHANGE_RATE = 0x3010000;

    // UX Prediction Resources
    public static final int PERF_RES_UX_PREDICT_GAME_MODE = 0x3004100;
    public static final int PERF_RES_UX_PREDICT_LOW_LATENCY = 0x3004000;

    // Constructor
    public PowerHalMgr() {
    }

    // Methods
    public boolean flushPriorityRules(int type) {
        return false;
    }

    public void mtkPowerHint(int hint, int data) {
    }

    public int perfCusLockHint(int hint, int duration) {
        return -1;
    }

    public int perfLockAcquire(int handle, int duration, int[] list) {
        return -1;
    }

    public void perfLockRelease(int handle) {
    }

    public void scnConfig(int handle, int cmd, int param1, int param2, int param3, int param4) {
    }

    public void scnDisable(int handle) {
    }

    public void scnEnable(int handle, int timeout) {
    }

    public int scnReg() {
        return -1;
    }

    public void scnUnreg(int handle) {
    }

    public boolean setPriorityByUid(int action, int uid) {
        return false;
    }
}
