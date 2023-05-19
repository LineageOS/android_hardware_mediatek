/*
 * Copyright 2020, The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "Gnss-main"

#include <android-base/logging.h>
#include <android/binder_manager.h>
#include <android/binder_process.h>
#include <log/log.h>
#include <pthread.h>

namespace aidl::android::hardware::gnss {
extern int aidl_gnss_main();
}

int main() {
    //// Register AIDL service
    ALOGE("Registering passthrough GNSS hal AIDL v1 service");
    ABinderProcess_setThreadPoolMaxThreadCount(1);
    ABinderProcess_startThreadPool();
    aidl::android::hardware::gnss::aidl_gnss_main();

    ABinderProcess_joinThreadPool();
    return EXIT_FAILURE;  // should not reach
}
