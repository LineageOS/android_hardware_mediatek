# MediaTek MSSI Overlays

## Overview

This directory provides generic overlays for MediaTek platforms.

It lets device maintainers and contributors keep only device-specific changes in their device overlays, ensuring everyone starts from the same feature baseline for their platform.

## Overlays list

```text
├── MssiCarrierConfigOverlay
│
├── MssiDeviceAsWebcamOverlay
│
├── MssiFrameworkOverlay
│
├── MssiFrameworkTelephonyOverlay
│
├── MssiFrameworkTelephony5gOverlay
│
├── MssiNetworkStackOverlay
│
├── MssiTelephonyOverlay
│
├── MssiWifiOverlay
│
├── MssiWifi6gOverlay
│
├── MssiWifiConnac3ChipsetOverlay
│
├── MssiWifiCountryOverlay
│
├── MssiWifiMultiStaOverlay
│
├── MssiWifiOweOverlay
│
└── MssiWifiQosOverlay
```

## Glossary

* **Helio platforms:** MediaTek SoCs sold under the Helio brand (split across the GXX and PXX series), all of which include a 4G modem on mobile variants and a CONNAC 1 chip.

* **Dimensity platforms:** MediaTek SoCs sold under the Dimensity brand (split across the 6XXX, 7XXX, 8XXX, and 9XXX series), all of which include a 5G modem on mobile variants and a CONNAC 2 or CONNAC 3 chip.

* **CONNAC:** MediaTek connectivity chip generation:
  * **CONNAC 1:** Base Wi-Fi 5 (802.11ac) features.
  * **CONNAC 2:** Base Wi-Fi 6 and 6E (802.11ax) features.
  * **CONNAC 3:** Base Wi-Fi 7 (802.11be) features.

## Overlays description

* **MssiCarrierConfigOverlay:** MSSI `CarrierConfig` overlay with default carrier values. Only enabled on targets that define `ENABLE_VENDOR_RIL_SERVICE` as `true`; intended as a reference or for early bring-up.

* **MssiDeviceAsWebcamOverlay:** MSSI `DeviceAsWebcam` overlay for the default ignored V4L2 node list.

* **MssiFrameworkOverlay:** MSSI `framework.jar` overlay for default base features.

* **MssiFrameworkTelephonyOverlay:** MSSI `framework.jar` overlay for base telephony features. Only enabled on targets that define `ENABLE_VENDOR_RIL_SERVICE` as `true`.

* **MssiFrameworkTelephony5gOverlay:** MSSI `framework.jar` overlay for 5G telephony features. Only enabled on Dimensity targets that define `ENABLE_VENDOR_RIL_SERVICE` as `true`.

* **MssiNetworkStackOverlay:** MSSI `NetworkStack`overlay for base networking features.

* **MssiTelephonyOverlay:** MSSI `TelephonyService` overlay for default MediaTek GBA and IMS packages. Only enabled on targets that define `ENABLE_VENDOR_RIL_SERVICE` as `true`.

* **MssiWifiOverlay:** MSSI `WifiCustomization` overlay for base Wi-Fi features.

* **MssiWifi6gOverlay:** MSSI `WifiCustomization` overlay for base Wi-Fi 6 and 6E (802.11ax) features. Only enabled on CONNAC 2 and CONNAC 3 chips.

* **MssiWifiConnac3ChipsetOverlay:** MSSI `WifiCustomization` overlay for base CONNAC 3 chip features, including Wi-Fi 7 (802.11be).

* **MssiWifiCountryOverlay:** MSSI `WifiCustomization` overlay for Wi-Fi country code changes. Only enabled on targets whose vendor is frozen on Android 16 or newer.

* **MssiWifiMultiStaOverlay:** MSSI `WifiCustomization` overlay for the Multi-STA feature. Only enabled on CONNAC 2 and CONNAC 3 chips.

* **MssiWifiOweOverlay:** MSSI `WifiCustomization` overlay for the SoftAP OWE feature. Only enabled on targets whose vendor is frozen on Android 16 or newer.

* **MssiWifiQosOverlay:** MSSI `WifiCustomization` overlay for the Quality of Service (QoS) feature. Only enabled on targets whose vendor is frozen on Android 14 or newer.
