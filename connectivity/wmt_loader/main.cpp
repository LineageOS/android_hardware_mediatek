/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#include <android-base/logging.h>
#include <android-base/properties.h>
#include <unistd.h>

#include <cstdlib>

#include "wmt.h"

namespace {

bool ParseOptions(int argc, char** argv, LauncherOptions* options) {
    int opt;
    while ((opt = getopt(argc, argv, "o:")) != -1) {
        switch (opt) {
            case 'o': {
                options->default_on = atoi(optarg);
                break;
            }
            default: {
                return false;
            }
        }
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    LauncherOptions options;
    if (!ParseOptions(argc, argv, &options)) {
        LOG(ERROR) << "usage: wmt_launcher [-m 3|4] [-b baud] [-o 0|1]";
        return EXIT_FAILURE;
    }

    // After a launcher-only restart, the driver is already up.
    if (android::base::GetProperty(kDriverReadyProp, "") == "yes") {
        LOG(INFO) << "WMT driver is already ready, skipping the loader phase";
    } else if (!RunLoader()) {
        return EXIT_FAILURE;
    }

    return RunLauncher(options);
}
