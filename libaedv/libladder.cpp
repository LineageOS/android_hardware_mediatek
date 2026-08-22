//
// SPDX-FileCopyrightText: The LineageOS Project
// SPDX-License-Identifier: Apache-2.0
//

#include <sys/types.h>
#include <string>
#include <vector>

extern "C" bool UnwindCurThreadBT(std::string* strBacktrace) {
    strBacktrace->clear();
    return false;
}

extern "C" bool UnwindThreadBT(pid_t tid, std::string* strBacktrace) {
    strBacktrace->clear();
    return false;
}

extern "C" bool UnwindCurProcessBT(std::string* strBacktrace) {
    strBacktrace->clear();
    return false;
}

extern "C" bool UnwindCurProcessBT_Vector(std::vector<std::string>* strBacktrace) {
    strBacktrace->clear();
    return false;
}
