#ifndef ORBIEDGE_JSON_LINE_HPP
#define ORBIEDGE_JSON_LINE_HPP

#include <cstdint>
#include <cstdio>
#include <cstring>

// Subset parser for the two Basilisk objects only. No JSON library.
// Sensor: sim_time_ns, soc, solar_scale, wheel_speed_rad_s[4], temp_c, heater_on.
// Mode: sim_time_ns, mode, wheel_index.

namespace Orbiedge {

struct JsonLine {
    enum class Kind : std::uint8_t { NONE = 0, SENSOR = 1, MODE = 2 };

    Kind kind;
    std::uint64_t sim_time_ns;
    double soc;
    double solar_scale;
    double wheel_speed_rad_s[4];
    double temp_c;
    bool heater_on;
    std::uint8_t mode;
    std::uint8_t wheel_index;

    static bool parse(const char* line, JsonLine& out);
    static int write_mode(char* dest, int dest_size, std::uint64_t sim_time_ns, const char* mode_name,
                          std::uint8_t wheel_index);
    static const char* mode_name(std::uint8_t mode);
};

inline const char* JsonLine::mode_name(std::uint8_t mode) {
    switch (mode) {
        case 0:
            return "NOMINAL";
        case 1:
            return "ISOLATE_RW";
        case 2:
            return "MOMENTUM_DUMP";
        case 3:
            return "LOAD_SHED";
        case 4:
            return "SAFE";
        default:
            return nullptr;
    }
}

inline int JsonLine::write_mode(char* dest, int dest_size, std::uint64_t sim_time_ns, const char* name,
                                std::uint8_t wheel_index) {
    if (dest == nullptr || dest_size <= 1 || name == nullptr) {
        return -1;
    }
    const int wrote = std::snprintf(dest, static_cast<std::size_t>(dest_size),
                                    "{\"sim_time_ns\":%llu,\"mode\":\"%s\",\"wheel_index\":%u}\n",
                                    static_cast<unsigned long long>(sim_time_ns), name,
                                    static_cast<unsigned>(wheel_index));
    if (wrote < 0 || wrote >= dest_size) {
        return -1;
    }
    return wrote;
}

inline bool JsonLine::parse(const char* line, JsonLine& out) {
    out = JsonLine{};
    if (line == nullptr) {
        return false;
    }

    struct Parser {
        const char* cursor;

        void skip() {
            while (*this->cursor == ' ' || *this->cursor == '\t' || *this->cursor == '\r' || *this->cursor == '\n') {
                this->cursor += 1;
            }
        }

        bool eat(char expected) {
            this->skip();
            if (*this->cursor != expected) {
                return false;
            }
            this->cursor += 1;
            return true;
        }

        static bool is_digit(char ch) { return ch >= '0' && ch <= '9'; }

        bool parse_u64(std::uint64_t& value) {
            this->skip();
            if (!is_digit(*this->cursor)) {
                return false;
            }
            std::uint64_t parsed = 0;
            while (is_digit(*this->cursor)) {
                const std::uint64_t digit = static_cast<std::uint64_t>(*this->cursor - '0');
                if (parsed > (static_cast<std::uint64_t>(~0ULL) - digit) / 10ULL) {
                    return false;
                }
                parsed = parsed * 10ULL + digit;
                this->cursor += 1;
            }
            value = parsed;
            return true;
        }

        bool parse_f64(double& value) {
            this->skip();
            int sign = 1;
            if (*this->cursor == '-' || *this->cursor == '+') {
                if (*this->cursor == '-') {
                    sign = -1;
                }
                this->cursor += 1;
            }
            if (!is_digit(*this->cursor) && *this->cursor != '.') {
                return false;
            }
            double parsed = 0.0;
            bool any = false;
            while (is_digit(*this->cursor)) {
                any = true;
                parsed = parsed * 10.0 + static_cast<double>(*this->cursor - '0');
                this->cursor += 1;
            }
            if (*this->cursor == '.') {
                this->cursor += 1;
                double place = 0.1;
                while (is_digit(*this->cursor)) {
                    any = true;
                    parsed += static_cast<double>(*this->cursor - '0') * place;
                    place *= 0.1;
                    this->cursor += 1;
                }
            }
            if (!any) {
                return false;
            }
            if (*this->cursor == 'e' || *this->cursor == 'E') {
                this->cursor += 1;
                int exponent_sign = 1;
                if (*this->cursor == '-' || *this->cursor == '+') {
                    if (*this->cursor == '-') {
                        exponent_sign = -1;
                    }
                    this->cursor += 1;
                }
                if (!is_digit(*this->cursor)) {
                    return false;
                }
                int exponent = 0;
                while (is_digit(*this->cursor)) {
                    exponent = exponent * 10 + (*this->cursor - '0');
                    if (exponent > 32) {
                        return false;
                    }
                    this->cursor += 1;
                }
                double scale = 1.0;
                for (int index = 0; index < exponent; ++index) {
                    scale *= 10.0;
                }
                if (exponent_sign < 0) {
                    parsed /= scale;
                } else {
                    parsed *= scale;
                }
            }
            value = sign < 0 ? -parsed : parsed;
            return true;
        }

        bool parse_bool(bool& value) {
            this->skip();
            if (std::strncmp(this->cursor, "true", 4) == 0) {
                this->cursor += 4;
                value = true;
                return true;
            }
            if (std::strncmp(this->cursor, "false", 5) == 0) {
                this->cursor += 5;
                value = false;
                return true;
            }
            return false;
        }

        bool parse_string(char* dest, int dest_size) {
            this->skip();
            if (*this->cursor != '"') {
                return false;
            }
            this->cursor += 1;
            int length = 0;
            while (*this->cursor != '\0' && *this->cursor != '"') {
                const unsigned char ch = static_cast<unsigned char>(*this->cursor);
                if (ch < 0x20 || ch == '\\') {
                    return false;
                }
                if (length + 1 >= dest_size) {
                    return false;
                }
                dest[length] = *this->cursor;
                length += 1;
                this->cursor += 1;
            }
            if (*this->cursor != '"') {
                return false;
            }
            this->cursor += 1;
            dest[length] = '\0';
            return true;
        }
    };

    Parser parser{line};
    if (!parser.eat('{')) {
        return false;
    }

    unsigned sensor_mask = 0;
    unsigned mode_mask = 0;
    bool saw_mode = false;
    bool saw_sensor_field = false;
    while (true) {
        parser.skip();
        if (*parser.cursor == '}') {
            parser.cursor += 1;
            break;
        }
        char key[64];
        if (!parser.parse_string(key, static_cast<int>(sizeof(key)))) {
            return false;
        }
        if (!parser.eat(':')) {
            return false;
        }
        if (std::strcmp(key, "sim_time_ns") == 0) {
            if ((sensor_mask & 1u) != 0 || !parser.parse_u64(out.sim_time_ns)) {
                return false;
            }
            sensor_mask |= 1u;
            mode_mask |= 1u;
        } else if (std::strcmp(key, "soc") == 0) {
            if ((sensor_mask & 2u) != 0 || !parser.parse_f64(out.soc)) {
                return false;
            }
            sensor_mask |= 2u;
            saw_sensor_field = true;
        } else if (std::strcmp(key, "solar_scale") == 0) {
            if ((sensor_mask & 4u) != 0 || !parser.parse_f64(out.solar_scale)) {
                return false;
            }
            sensor_mask |= 4u;
            saw_sensor_field = true;
        } else if (std::strcmp(key, "temp_c") == 0) {
            if ((sensor_mask & 8u) != 0 || !parser.parse_f64(out.temp_c)) {
                return false;
            }
            sensor_mask |= 8u;
            saw_sensor_field = true;
        } else if (std::strcmp(key, "heater_on") == 0) {
            if ((sensor_mask & 16u) != 0 || !parser.parse_bool(out.heater_on)) {
                return false;
            }
            sensor_mask |= 16u;
            saw_sensor_field = true;
        } else if (std::strcmp(key, "wheel_speed_rad_s") == 0) {
            if ((sensor_mask & 32u) != 0 || !parser.eat('[')) {
                return false;
            }
            for (int wheel = 0; wheel < 4; ++wheel) {
                if (wheel > 0 && !parser.eat(',')) {
                    return false;
                }
                if (!parser.parse_f64(out.wheel_speed_rad_s[wheel])) {
                    return false;
                }
            }
            if (!parser.eat(']')) {
                return false;
            }
            sensor_mask |= 32u;
            saw_sensor_field = true;
        } else if (std::strcmp(key, "mode") == 0) {
            char name[32];
            if (saw_mode || !parser.parse_string(name, static_cast<int>(sizeof(name)))) {
                return false;
            }
            bool matched = false;
            for (std::uint8_t index = 0; index < 5; ++index) {
                if (std::strcmp(name, mode_name(index)) == 0) {
                    out.mode = index;
                    matched = true;
                    break;
                }
            }
            if (!matched) {
                return false;
            }
            saw_mode = true;
            mode_mask |= 2u;
        } else if (std::strcmp(key, "wheel_index") == 0) {
            std::uint64_t wheel = 0;
            if ((mode_mask & 4u) != 0 || !parser.parse_u64(wheel) || wheel > 255ULL) {
                return false;
            }
            out.wheel_index = static_cast<std::uint8_t>(wheel);
            mode_mask |= 4u;
        } else {
            return false;
        }
        parser.skip();
        if (*parser.cursor == ',') {
            parser.cursor += 1;
            continue;
        }
        if (*parser.cursor == '}') {
            parser.cursor += 1;
            break;
        }
        return false;
    }
    parser.skip();
    if (*parser.cursor != '\0') {
        return false;
    }
    if (saw_mode && saw_sensor_field) {
        return false;
    }
    if (saw_sensor_field) {
        if (sensor_mask != (1u | 2u | 4u | 8u | 16u | 32u)) {
            return false;
        }
        out.kind = Kind::SENSOR;
        return true;
    }
    if (saw_mode) {
        if (mode_mask != (1u | 2u | 4u)) {
            return false;
        }
        out.kind = Kind::MODE;
        return true;
    }
    return false;
}

}  // namespace Orbiedge

#endif
