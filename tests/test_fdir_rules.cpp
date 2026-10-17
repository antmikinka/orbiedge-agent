#include "Orbiedge/FdirRules/FdirRules.hpp"

#include <cmath>
#include <cstdio>

namespace {

int g_failed = 0;

void expect(bool condition, const char* message) {
    if (!condition) {
        std::printf("FAIL %s\n", message);
        g_failed += 1;
    }
}

const char* mode_name(Orbiedge::FdirRules::Mode mode) {
    switch (mode) {
        case Orbiedge::FdirRules::Mode::NOMINAL:
            return "NOMINAL";
        case Orbiedge::FdirRules::Mode::ISOLATE_RW:
            return "ISOLATE_RW";
        case Orbiedge::FdirRules::Mode::MOMENTUM_DUMP:
            return "MOMENTUM_DUMP";
        case Orbiedge::FdirRules::Mode::LOAD_SHED:
            return "LOAD_SHED";
        case Orbiedge::FdirRules::Mode::SAFE:
            return "SAFE";
        default:
            return "?";
    }
}

Orbiedge::FdirRules::Sample sample_at(std::uint64_t sim_time_ns) {
    Orbiedge::FdirRules::Sample sample{};
    sample.sim_time_ns = sim_time_ns;
    sample.soc = 0.8;
    sample.solar_scale = 1.0;
    sample.temp_c = 20.0;
    sample.heater_on = false;
    sample.valid = true;
    return sample;
}

void test_temp_critical() {
    const int before = g_failed;
    Orbiedge::FdirRules rules;
    Orbiedge::FdirRules::Decision decision{};
    for (int index = 0; index < Orbiedge::FdirRules::TEMP_CRIT_SAMPLES; ++index) {
        Orbiedge::FdirRules::Sample sample = sample_at(static_cast<std::uint64_t>(index) * Orbiedge::FdirRules::STEP_NS);
        sample.temp_c = 70.0;
        decision = rules.tick(sample, nullptr);
    }
    expect(decision.mode == Orbiedge::FdirRules::Mode::SAFE, "50 temp 70 C samples");
    if (g_failed == before) {
        std::printf("PASS 50 temp 70 C -> %s\n", mode_name(decision.mode));
    }
}

void test_nominal() {
    const int before = g_failed;
    Orbiedge::FdirRules rules;
    Orbiedge::FdirRules::Decision decision{};
    for (int index = 0; index < 20; ++index) {
        decision = rules.tick(sample_at(static_cast<std::uint64_t>(index) * Orbiedge::FdirRules::STEP_NS), nullptr);
        expect(decision.mode == Orbiedge::FdirRules::Mode::NOMINAL, "nominal sample left NOMINAL");
    }
    if (g_failed == before) {
        std::printf("PASS 20 nominal -> %s\n", mode_name(decision.mode));
    }
}

void test_soc_critical() {
    const int before = g_failed;
    Orbiedge::FdirRules rules;
    Orbiedge::FdirRules::Decision decision{};
    for (int index = 0; index < Orbiedge::FdirRules::SOC_SAMPLES; ++index) {
        Orbiedge::FdirRules::Sample sample =
            sample_at(static_cast<std::uint64_t>(index) * Orbiedge::FdirRules::SOC_PERIOD_NS);
        sample.soc = 0.10;
        decision = rules.tick(sample, nullptr);
    }
    expect(decision.mode == Orbiedge::FdirRules::Mode::SAFE, "SOC 0.10 five times at 1 Hz");
    if (g_failed == before) {
        std::printf("PASS SOC 0.10 at 1 Hz x5 -> %s\n", mode_name(decision.mode));
    }
}

void test_overspeed_window() {
    const int before = g_failed;
    Orbiedge::FdirRules rules;
    Orbiedge::FdirRules::Decision decision{};
    const double speed = 0.95 * Orbiedge::FdirRules::WHEEL_MAX_RAD_S;
    for (int index = 0; index < 30; ++index) {
        const std::uint64_t sim_time_ns = static_cast<std::uint64_t>(index) * Orbiedge::FdirRules::STEP_NS;
        Orbiedge::FdirRules::Sample sample = sample_at(sim_time_ns);
        sample.wheel_speed_rad_s[0] = speed;
        Orbiedge::FdirRules::Proposal proposal{};
        const Orbiedge::FdirRules::Proposal* proposal_ptr = nullptr;
        if (index == Orbiedge::FdirRules::OVERSPEED_SAMPLES - 1) {
            proposal.opcode = Orbiedge::FdirRules::OPCODE_PROPOSE;
            proposal.action = Orbiedge::FdirRules::Mode::LOAD_SHED;
            proposal.wheel_index = 0;
            proposal.confidence = 0.5f;
            proposal.sim_time_ns = sim_time_ns;
            proposal.wall_time_ns = 1000;
            proposal_ptr = &proposal;
        }
        decision = rules.tick(sample, proposal_ptr);
        if (index == Orbiedge::FdirRules::OVERSPEED_SAMPLES - 1) {
            expect(decision.disposition == Orbiedge::FdirRules::Disposition::REJECT, "LOAD_SHED during overspeed warning");
            expect(decision.mode == Orbiedge::FdirRules::Mode::NOMINAL, "window still open at persistence");
            expect(decision.severity == Orbiedge::FdirRules::Severity::WARNING, "overspeed severity");
            expect(decision.default_mode == Orbiedge::FdirRules::Mode::MOMENTUM_DUMP, "overspeed default");
            const std::uint8_t dump = Orbiedge::FdirRules::mode_bit(Orbiedge::FdirRules::Mode::MOMENTUM_DUMP);
            const std::uint8_t safe = Orbiedge::FdirRules::mode_bit(Orbiedge::FdirRules::Mode::SAFE);
            const std::uint8_t shed = Orbiedge::FdirRules::mode_bit(Orbiedge::FdirRules::Mode::LOAD_SHED);
            expect((decision.eligible & dump) != 0 && (decision.eligible & safe) != 0, "overspeed eligible set");
            expect((decision.eligible & shed) == 0, "LOAD_SHED is not eligible");
        }
        if (index == 28) {
            expect(decision.mode == Orbiedge::FdirRules::Mode::NOMINAL, "mode waits for the 2 s window");
        }
    }
    expect(decision.mode == Orbiedge::FdirRules::Mode::MOMENTUM_DUMP, "window expiry applies MOMENTUM_DUMP");
    if (g_failed == before) {
        std::printf("PASS overspeed 10 samples, LOAD_SHED rejected, window -> %s\n", mode_name(decision.mode));
    }
}

void test_invalid_holds() {
    const int before = g_failed;
    Orbiedge::FdirRules rules;
    for (int index = 0; index < Orbiedge::FdirRules::TEMP_CRIT_SAMPLES - 1; ++index) {
        Orbiedge::FdirRules::Sample sample = sample_at(static_cast<std::uint64_t>(index) * Orbiedge::FdirRules::STEP_NS);
        sample.temp_c = 70.0;
        const Orbiedge::FdirRules::Decision decision = rules.tick(sample, nullptr);
        expect(decision.mode == Orbiedge::FdirRules::Mode::NOMINAL, "not SAFE before 50 valid samples");
    }
    Orbiedge::FdirRules::Sample invalid =
        sample_at(static_cast<std::uint64_t>(Orbiedge::FdirRules::TEMP_CRIT_SAMPLES) * Orbiedge::FdirRules::STEP_NS);
    invalid.temp_c = 70.0;
    invalid.valid = false;
    Orbiedge::FdirRules::Decision held = rules.tick(invalid, nullptr);
    expect(held.mode == Orbiedge::FdirRules::Mode::NOMINAL, "invalid input holds mode");
    expect(held.disposition == Orbiedge::FdirRules::Disposition::HOLD_INVALID, "HOLD_INVALID");
    expect(!held.sensor_valid, "sensor invalid");

    Orbiedge::FdirRules::Sample resume = sample_at(
        static_cast<std::uint64_t>(Orbiedge::FdirRules::TEMP_CRIT_SAMPLES + 1) * Orbiedge::FdirRules::STEP_NS);
    resume.temp_c = 70.0;
    const Orbiedge::FdirRules::Decision decision = rules.tick(resume, nullptr);
    expect(decision.mode == Orbiedge::FdirRules::Mode::SAFE, "invalid sample did not count or reset persistence");
    if (g_failed == before) {
        std::printf("PASS invalid input holds mode\n");
    }
}

void test_population_stddev() {
    const int before = g_failed;
    const double values[2] = {0.0, 2.0};
    const double deviation = Orbiedge::FdirRules::population_stddev(values, 2);
    expect(std::fabs(deviation - 1.0) < 1e-12, "population stddev divides by N");
    const double flat[4] = {3.0, 3.0, 3.0, 3.0};
    expect(Orbiedge::FdirRules::population_stddev(flat, 4) == 0.0, "zero stddev");
    if (g_failed == before) {
        std::printf("PASS population stddev divides by N\n");
    }
}

void test_temp_warn_does_not_change_mode() {
    const int before = g_failed;
    Orbiedge::FdirRules rules;
    Orbiedge::FdirRules::Decision decision{};
    for (int index = 0; index < Orbiedge::FdirRules::TEMP_WARN_SAMPLES; ++index) {
        Orbiedge::FdirRules::Sample sample = sample_at(static_cast<std::uint64_t>(index) * Orbiedge::FdirRules::STEP_NS);
        sample.temp_c = 50.0;
        decision = rules.tick(sample, nullptr);
    }
    expect(decision.mode == Orbiedge::FdirRules::Mode::NOMINAL, "temp warning is event only");
    expect(decision.severity == Orbiedge::FdirRules::Severity::NONE, "temp warning severity NONE");
    expect(decision.temp_warn_event, "temp warning event");
    if (g_failed == before) {
        std::printf("PASS temp warning 100 samples stays NOMINAL\n");
    }
}

void test_isolate_clear() {
    const int before = g_failed;
    Orbiedge::FdirRules rules;
    Orbiedge::FdirRules::Decision decision{};
    for (int index = 0; index < 40; ++index) {
        Orbiedge::FdirRules::Sample sample = sample_at(static_cast<std::uint64_t>(index) * Orbiedge::FdirRules::STEP_NS);
        sample.wheel_speed_rad_s[0] = (index % 2 == 0) ? 0.0 : 4.0;
        decision = rules.tick(sample, nullptr);
    }
    expect(decision.mode == Orbiedge::FdirRules::Mode::ISOLATE_RW, "noisy wheel isolates");
    expect(decision.wheel_index == 0, "isolate wheel 0");
    rules.clear();
    expect(rules.mode() == Orbiedge::FdirRules::Mode::NOMINAL, "clear drops sticky isolate");
    Orbiedge::FdirRules::Sample again = sample_at(40 * Orbiedge::FdirRules::STEP_NS);
    again.wheel_speed_rad_s[0] = 4.0;
    decision = rules.tick(again, nullptr);
    expect(decision.mode == Orbiedge::FdirRules::Mode::NOMINAL, "persistence restarts after clear");
    if (g_failed == before) {
        std::printf("PASS ISOLATE_RW sticky until clear\n");
    }
}

void test_dump_rise() {
    const int before = g_failed;
    Orbiedge::FdirRules rules;
    const double speed = 0.95 * Orbiedge::FdirRules::WHEEL_MAX_RAD_S;
    std::uint64_t sim_time_ns = 0;
    for (int index = 0; index < 30; ++index) {
        Orbiedge::FdirRules::Sample sample = sample_at(sim_time_ns);
        sample.wheel_speed_rad_s[0] = speed;
        rules.tick(sample, nullptr);
        sim_time_ns += Orbiedge::FdirRules::STEP_NS;
    }
    expect(rules.mode() == Orbiedge::FdirRules::Mode::MOMENTUM_DUMP, "dump before rise");
    Orbiedge::FdirRules::Sample rising = sample_at(sim_time_ns - Orbiedge::FdirRules::STEP_NS + Orbiedge::FdirRules::DUMP_RISE_NS);
    rising.wheel_speed_rad_s[0] = speed + 1.0;
    const Orbiedge::FdirRules::Decision decision = rules.tick(rising, nullptr);
    expect(decision.mode == Orbiedge::FdirRules::Mode::SAFE, "dump rise 30 s");
    if (g_failed == before) {
        std::printf("PASS dump rise 30 s -> SAFE\n");
    }
}

void test_load_shed_quiet() {
    const int before = g_failed;
    Orbiedge::FdirRules rules;
    std::uint64_t sim_time_ns = 0;
    for (int index = 0; index < Orbiedge::FdirRules::SOC_SAMPLES; ++index) {
        Orbiedge::FdirRules::Sample sample = sample_at(sim_time_ns);
        sample.soc = 0.20;
        rules.tick(sample, nullptr);
        sim_time_ns += Orbiedge::FdirRules::SOC_PERIOD_NS;
    }
    Orbiedge::FdirRules::Sample warning = sample_at((Orbiedge::FdirRules::SOC_SAMPLES - 1) * Orbiedge::FdirRules::SOC_PERIOD_NS +
                                                    Orbiedge::FdirRules::PROPOSAL_WINDOW_NS);
    warning.soc = 0.20;
    Orbiedge::FdirRules::Decision decision = rules.tick(warning, nullptr);
    expect(decision.mode == Orbiedge::FdirRules::Mode::LOAD_SHED, "SOC warning default");
    Orbiedge::FdirRules::Sample quiet = sample_at(warning.sim_time_ns + Orbiedge::FdirRules::STEP_NS);
    quiet.soc = 0.80;
    decision = rules.tick(quiet, nullptr);
    expect(decision.mode == Orbiedge::FdirRules::Mode::LOAD_SHED, "quiet dwell not finished");
    quiet.sim_time_ns += Orbiedge::FdirRules::QUIET_DWELL_NS;
    decision = rules.tick(quiet, nullptr);
    expect(decision.mode == Orbiedge::FdirRules::Mode::NOMINAL, "60 s quiet dwell");
    if (g_failed == before) {
        std::printf("PASS LOAD_SHED returns NOMINAL after quiet dwell\n");
    }
}

void test_filter() {
    const int before = g_failed;
    Orbiedge::FdirRules rules;
    expect(rules.screen(Orbiedge::FdirRules::OPCODE_CLEAR, 0, 0, 0, 0, 1.0f) == Orbiedge::FdirRules::FilterResult::DROP_CLEAR,
           "drop FDIR.CLEAR");
    expect(rules.screen("FDIR.SET_PARAM", 0, 0, 0, 0, 1.0f) == Orbiedge::FdirRules::FilterResult::DROP_OPCODE,
           "drop other opcode");
    expect(rules.screen(Orbiedge::FdirRules::OPCODE_PROPOSE, 0, 0, 0, 0, 1.0f) == Orbiedge::FdirRules::FilterResult::PASS,
           "pass propose");
    expect(rules.screen(Orbiedge::FdirRules::OPCODE_PROPOSE, 0, 0, 1000, 0, 1.0f) == Orbiedge::FdirRules::FilterResult::DROP_RATE,
           "one proposal per 2 s wall");
    expect(rules.screen(Orbiedge::FdirRules::OPCODE_PROPOSE, 0, Orbiedge::FdirRules::PROPOSAL_STALE_NS + 1,
                        Orbiedge::FdirRules::PROPOSAL_WINDOW_NS, 0, 1.0f) == Orbiedge::FdirRules::FilterResult::DROP_STALE,
           "past proposal is stale");
    expect(rules.screen(Orbiedge::FdirRules::OPCODE_PROPOSE, Orbiedge::FdirRules::PROPOSAL_STALE_NS + 1, 0,
                        Orbiedge::FdirRules::PROPOSAL_WINDOW_NS * 2, 0, 1.0f) ==
               Orbiedge::FdirRules::FilterResult::DROP_STALE,
           "future proposal is stale");
    if (g_failed == before) {
        std::printf("PASS agent filter\n");
    }
}

}  // namespace

int main() {
    test_temp_critical();
    test_nominal();
    test_soc_critical();
    test_overspeed_window();
    test_invalid_holds();
    test_population_stddev();
    test_temp_warn_does_not_change_mode();
    test_isolate_clear();
    test_dump_rise();
    test_load_shed_quiet();
    test_filter();
    if (g_failed != 0) {
        std::printf("FAILED %d\n", g_failed);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
