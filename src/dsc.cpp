#include <map>

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

#include <PicoSyslog.h>
#include <PicoMQTT.h>
#include <PicoUtils.h>
#include <dscKeybusInterface.h>

String hostname = "dsc";
PicoSyslog::Logger syslog("dsc");

PicoUtils::PinInput button(D7, true);
PicoUtils::ResetButton reset_button(button);

PicoUtils::PinOutput wifi_led(D4, true);
PicoUtils::PinOutput keybus_led(D3, true);

PicoUtils::WiFiControlSmartConfig wifi_control(wifi_led);
PicoUtils::Blink keybus_led_blinker(keybus_led, 0, 131);

dscKeybusInterface dsc(D1, D2, D8);

PicoMQTT::Client mqtt;

unsigned int partition;
std::map<int, String> zones;
std::map<int, String> pgm;

const String board_id(ESP.getChipId(), HEX);
const String topic_prefix = "keybus/" + board_id + "/";

const char CONFIG_FILE[] PROGMEM = "/config.json";

void update_bool(const bool & value, bool & changed, const bool force, const String & topic) {
    if (!changed && !force)
        return;

    changed = false;
    syslog.printf("%s: %s\n", topic.c_str(), value ? "ON": "OFF");
    mqtt.publish(topic_prefix + topic, value ? "ON": "OFF", 0, true);
}

void update_bitmask(const byte values[], bool & global_changed, byte changed[], std::map<int, String> ids, const bool force, const String & topic) {
    if (!global_changed && !force)
        return;

    global_changed = false;

    for (const auto & element : ids) {
        const auto idx = element.first - 1;
        const unsigned int group = idx >> 3;
        const unsigned int position = idx & 0x7;
        const byte mask = 1 << position;
        if ((changed[group] & mask) || force) {
            changed[group] &= ~mask;
            mqtt.publish(topic_prefix + topic + String(element.first), values[group] & mask ? "ON": "OFF", 0, true);
        }
    }
}

const char * get_state() {

    if (dsc.alarm[partition] || dsc.fire[partition]) {
        return "triggered";
    }

    if (dsc.entryDelay[partition]) {
        return "pending";
    }

    if (dsc.exitDelay[partition]) {
        return "arming";
    }

    if (dsc.armed[partition]) {
        if ((dsc.armedAway[partition] || dsc.armedStay[partition]) && dsc.noEntryDelay[partition])
            return "armed_night";
        else if (dsc.armedAway[partition])
            return "armed_away";
        else if (dsc.armedStay[partition])
            return "armed_home";
    }

    return "disarmed";
}

void update(const bool force=false) {
    if (!dsc.statusChanged && !force)
        return;

    dsc.statusChanged = false;

    update_bool(dsc.trouble, dsc.troubleChanged, force, "trouble");
    update_bool(dsc.batteryTrouble, dsc.batteryChanged, force, "battery_trouble");
    update_bool(dsc.powerTrouble, dsc.powerChanged, force, "power_trouble");

    if (!dsc.disabled[partition]) {
        if (force
            || dsc.alarmChanged[partition]
            || dsc.armedChanged[partition]
            || dsc.entryDelayChanged[partition]
            || dsc.exitDelayChanged[partition]
            || dsc.fireChanged[partition]
            ) {

            dsc.alarmChanged[partition] = false;
            dsc.armedChanged[partition] = false;
            dsc.entryDelayChanged[partition] = false;
            dsc.exitDelayChanged[partition] = false;
            dsc.fireChanged[partition] = false;

            mqtt.publish(topic_prefix + "alarm", get_state(), 0, true);
        }
    }

    update_bitmask(dsc.openZones, dsc.openZonesStatusChanged, dsc.openZonesChanged, zones, force, "zone/motion/");
    update_bitmask(dsc.alarmZones, dsc.alarmZonesStatusChanged, dsc.alarmZonesChanged, zones, force, "zone/alarm/");
    update_bitmask(dsc.pgmOutputs, dsc.pgmOutputsStatusChanged, dsc.pgmOutputsChanged, pgm, force, "pgm/");
}

PicoUtils::Watch<bool> connected_watch(
        []{ return dsc.keybusConnected; },
        [](bool connected) {
            keybus_led_blinker.set_pattern(connected ? 1 : 0b10);
            mqtt.publish(topic_prefix + "connected", connected ? "ON" : "OFF", 0, true);
        });

namespace HomeAssistant {

const String board_unique_id = "keybus-" + board_id;

JsonDocument get_device() {
    JsonDocument device;
    device["name"] = "DSC Alarm";
    device["identifiers"][0] = board_unique_id;
    device["configuration_url"] = "http://" + WiFi.localIP().toString();
    device["manufacturer"] = "mlesniew";
    device["model"] = "Keybus";
    device["sw_version"] = __DATE__ " " __TIME__;
    return device;
}

void autodiscovery() {
    syslog.println("Sending Home Assistant autodiscovery messages...");

    {
        const auto unique_id = board_unique_id + "-alarm";

        JsonDocument json;

        json["unique_id"] = unique_id;
        json["availability_topic"] = mqtt.will.topic;

        json["code"] = "REMOTE_CODE";
        json["code_arm_required"] = false;
        json["code_disarm_required"] = true;
        json["state_topic"] = topic_prefix + "alarm";
        json["command_topic"] = topic_prefix + "alarm/command";
        json["command_template"] = "{ \"action\": \"{{ action }}\", \"code\": \"{{ code }}\" }";
        json["supported_features"][0] = "arm_home";
        json["supported_features"][1] = "arm_away";

        json["device"] = get_device();

        const String disco_topic = "homeassistant/alarm_control_panel/" + unique_id + "/config";
        auto publish = mqtt.begin_publish(disco_topic, measureJson(json), 0, true);
        serializeJson(json, publish);
        publish.send();
    }

    for (const auto & zone : zones) {
        const auto & zone_id = zone.first;
        const auto & zone_name = zone.second;

        const auto unique_id_base = board_unique_id + "-zone-" + String(zone_id);

        JsonDocument device;
        device["name"] = String(zone_name);
        device["suggested_area"] = zone_name;
        device["identifiers"][0] = unique_id_base + "-motion-sensor";
        device["via_device"] = board_unique_id;

        {
            const auto unique_id = unique_id_base + "-motion";

            JsonDocument json;

            json["unique_id"] = unique_id;
            json["availability_topic"] = mqtt.will.topic;
            json["device_class"] = "motion";
            json["state_topic"] = topic_prefix + "zone/motion/" + String(zone_id);
            json["device"] = device;

            const String disco_topic = "homeassistant/binary_sensor/" + unique_id + "/config";
            auto publish = mqtt.begin_publish(disco_topic, measureJson(json), 0, true);
            serializeJson(json, publish);
            publish.send();
        }

        {
            const auto unique_id = unique_id_base + "-alarm";

            JsonDocument json;

            json["unique_id"] = unique_id;
            json["name"] = "Alarm";
            json["availability_topic"] = mqtt.will.topic;
            json["device_class"] = "safety";
            json["state_topic"] = topic_prefix + "zone/alarm/" + String(zone_id);
            json["device"] = device;

            const String disco_topic = "homeassistant/binary_sensor/" + unique_id + "/config";
            auto publish = mqtt.begin_publish(disco_topic, measureJson(json), 0, true);
            serializeJson(json, publish);
            publish.send();
        }

    }

    for (const auto & output : pgm) {
        const auto & id = output.first;
        const auto & name = output.second;

        const auto unique_id = board_unique_id + "-pgm-" + String(id);

        JsonDocument json;

        json["unique_id"] = unique_id;
        json["name"] = name;
        json["availability_topic"] = mqtt.will.topic;
        json["state_topic"] = topic_prefix + "pgm/" + String(id);

        json["device"] = get_device();

        const String disco_topic = "homeassistant/binary_sensor/" + unique_id + "/config";
        auto publish = mqtt.begin_publish(disco_topic, measureJson(json), 0, true);
        serializeJson(json, publish);
        publish.send();
    }

    struct BinarySensor {
        const char * name;
        const char * friendly_name;
        const char * device_class;
    };

    static const BinarySensor binary_sensors[] = {
        {"trouble", "Trouble", "problem"},
        {"battery_trouble", "Battery", "problem"},
        {"power_trouble", "Power", "problem"},
        {"connected", "Keybus", "connectivity"},
    };

    for (const auto & binary_sensor : binary_sensors) {
        const auto unique_id = board_unique_id + "-" + binary_sensor.name;
        JsonDocument json;
        json["unique_id"] = unique_id;
        json["object_id"] = String("keybus_") + binary_sensor.name;
        json["name"] = binary_sensor.friendly_name;
        json["device_class"] = binary_sensor.device_class;
        json["entity_category"] = "diagnostic";
        json["availability_topic"] = mqtt.will.topic;
        json["state_topic"] = topic_prefix + binary_sensor.name;

        json["device"] = get_device();

        const String disco_topic = "homeassistant/binary_sensor/" + unique_id + "/config";
        auto publish = mqtt.begin_publish(disco_topic, measureJson(json), 0, true);
        serializeJson(json, publish);
        publish.send();
    }
}

}

namespace settings {

void load(const JsonDocument & config) {
    {
        const auto mqtt_config = config["mqtt"];
        mqtt.host = mqtt_config["host"] | "";
        mqtt.port = mqtt_config["port"] | 1883;
        mqtt.username = mqtt_config["username"] | "";
        mqtt.password = mqtt_config["password"] | "";
    }

    for (JsonPairConst kv : config["zones"].as<JsonObjectConst>()) {
        zones[String(kv.key().c_str()).toInt()] = kv.value().as<String>();
    }

    for (JsonPairConst kv : config["pgm"].as<JsonObjectConst>()) {
        pgm[String(kv.key().c_str()).toInt()] = kv.value().as<String>();
    }

    syslog.server = config["syslog"] | "";
    hostname = config["hostname"] | "dsc";
    partition = config["partition"] | 1;
}

};

void setup() {
    wifi_led.init();
    keybus_led.init();
    keybus_led_blinker.set_pattern(0b10);
    PicoUtils::BackgroundBlinker blinker(keybus_led_blinker);

    Serial.begin(115200);

    Serial.println(F("\n\n"
                     "Keybus " __DATE__ " " __TIME__ "\n"
                     "\n\n"
                    ));

    reset_button.init();
    wifi_control.init(button);
    wifi_control.get_connectivity_level = []{
        return mqtt.connected() ? 2 : 1;
    };

    LittleFS.begin();
    {
        const auto config = PicoUtils::JsonConfigFile<JsonDocument>(LittleFS, FPSTR(CONFIG_FILE));
        settings::load(config);
    }

    mqtt.client_id = "keybus-" + board_id;
    mqtt.will.topic = "keybus/" + board_id + "/availability";
    mqtt.will.payload = "offline";
    mqtt.will.retain = true;

    mqtt.connected_callback = [] {
        // send autodiscovery messages
        HomeAssistant::autodiscovery();

        // notify about availability
        mqtt.publish(mqtt.will.topic, "online", 0, true);

        // notify about the current state
        update(true);

        connected_watch.fire();
    };

    mqtt.begin();

    ArduinoOTA.setHostname(hostname.c_str());
    ArduinoOTA.onStart([] { dsc.stop(); });
    ArduinoOTA.onError([](uint8_t) { dsc.begin(); });
    ArduinoOTA.begin();

    dsc.writePartition = partition;
    dsc.begin();
}

PicoUtils::PeriodicRun healthcheck(15, [] {
    static PicoUtils::Stopwatch last_healthy;

    if ((WiFi.status() == WL_CONNECTED) && (dsc.keybusConnected)) {
        last_healthy.reset();
    }

    if ((last_healthy.elapsed() >= 15 * 60)) {
        syslog.println(F("Healthcheck failing for too long.  Reset..."));
        ESP.reset();
    }
});

void loop() {
    ArduinoOTA.handle();
    dsc.loop();
    update();
    mqtt.loop();
    wifi_control.tick();
    healthcheck.tick();
    keybus_led_blinker.tick();
    connected_watch.tick();
}
