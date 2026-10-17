#ifndef ORBIEDGE_FDIR_RULES_HPP
#define ORBIEDGE_FDIR_RULES_HPP

#include <cmath>
#include <cstdint>
#include <cstring>

// Header-only C++17 rules. No F Prime types. tests/test_fdir_rules.cpp links this header only.
// Outcomes match ARCHITECTURE.md: BusMonitor predicates, FdirManager tick rules, uplink filter.
// Population standard deviation divides by N. clear() restarts persistence and drops sticky modes.
// The advisor rate limit is not cleared by clear().

namespace Orbiedge {

class FdirRules {
  public:
    static constexpr std::uint64_t STEP_NS = 100000000ULL;
    static constexpr int WHEEL_COUNT = 4;
    static constexpr double WHEEL_MAX_RAD_S = 628.3185307179587;
    static constexpr double OVERSPEED_FRACTION = 0.9;
    static constexpr int OVERSPEED_SAMPLES = 10;
    static constexpr std::uint64_t DUMP_RISE_NS = 30000000000ULL;
    static constexpr double STDDEV_HIGH = 1.0;
    static constexpr double STDDEV_QUIET = 0.2;
    static constexpr int STDDEV_SAMPLES = 20;
    static constexpr double SOC_WARN = 0.30;
    static constexpr double SOC_CRIT = 0.15;
    static constexpr int SOC_SAMPLES = 5;
    static constexpr double TEMP_WARN_C = 40.0;
    static constexpr double TEMP_CRIT_C = 60.0;
    static constexpr int TEMP_CRIT_SAMPLES = 50;
    static constexpr int TEMP_WARN_SAMPLES = 100;
    static constexpr std::uint64_t QUIET_DWELL_NS = 60000000000ULL;
    static constexpr std::uint64_t PROPOSAL_WINDOW_NS = 2000000000ULL;
    static constexpr std::uint64_t PROPOSAL_STALE_NS = 5000000000ULL;
    static constexpr std::uint64_t SOC_PERIOD_NS = 1000000000ULL;
    static constexpr int STDDEV_CAPACITY = 64;
    static constexpr const char* OPCODE_PROPOSE = "AGENT.PROPOSE";
    static constexpr const char* OPCODE_CLEAR = "FDIR.CLEAR";

    enum class Mode : std::uint8_t {
        NOMINAL = 0,
        ISOLATE_RW = 1,
        MOMENTUM_DUMP = 2,
        LOAD_SHED = 3,
        SAFE = 4
    };

    enum class Severity : std::uint8_t { NONE = 0, WARNING = 1, CRITICAL = 2 };

    enum class Disposition : std::uint8_t {
        NONE = 0,
        AGREE = 1,
        DISAGREE = 2,
        LATE = 3,
        REJECT = 4,
        HOLD_INVALID = 5
    };

    enum class FilterResult : std::uint8_t {
        PASS = 0,
        DROP_OPCODE = 1,
        DROP_CLEAR = 2,
        DROP_STALE = 3,
        DROP_RATE = 4,
        DROP_ARGS = 5
    };

    struct Sample {
        std::uint64_t sim_time_ns;
        double soc;
        double solar_scale;
        double wheel_speed_rad_s[WHEEL_COUNT];
        double temp_c;
        bool heater_on;
        bool valid;
    };

    struct Proposal {
        const char* opcode;
        Mode action;
        std::uint8_t wheel_index;
        float confidence;
        std::uint64_t sim_time_ns;
        std::uint64_t wall_time_ns;
    };

    struct Thresholds {
        double wheel_max_rad_s = WHEEL_MAX_RAD_S;
        double overspeed_fraction = OVERSPEED_FRACTION;
        int overspeed_samples = OVERSPEED_SAMPLES;
        std::uint64_t dump_rise_ns = DUMP_RISE_NS;
        double stddev_high = STDDEV_HIGH;
        double stddev_quiet = STDDEV_QUIET;
        int stddev_samples = STDDEV_SAMPLES;
        double soc_warn = SOC_WARN;
        double soc_crit = SOC_CRIT;
        int soc_samples = SOC_SAMPLES;
        double temp_warn_c = TEMP_WARN_C;
        double temp_crit_c = TEMP_CRIT_C;
        int temp_crit_samples = TEMP_CRIT_SAMPLES;
        int temp_warn_samples = TEMP_WARN_SAMPLES;
        std::uint64_t quiet_dwell_ns = QUIET_DWELL_NS;
    };

    struct FaultRecord {
        std::uint64_t sim_time_ns;
        std::uint8_t eligible;
        Mode default_mode;
        Severity severity;
        std::uint8_t wheel_index;
        bool sensor_valid;
        bool quiet;
        bool temp_warn_event;
        double soc;
        double solar_scale;
        double temp_c;
        double wheel_speed_rad_s[WHEEL_COUNT];
        double wheel_stddev[WHEEL_COUNT];
    };

    struct Decision {
        Mode mode;
        Severity severity;
        std::uint8_t eligible;
        Mode default_mode;
        Disposition disposition;
        std::uint8_t wheel_index;
        bool sensor_valid;
        bool temp_warn_event;
        std::uint64_t sim_time_ns;
        Mode last_proposal;
    };

    // Bit i is Mode value i. Overspeed is MOMENTUM_DUMP|SAFE. Isolate is ISOLATE_RW|NOMINAL.
    static std::uint8_t mode_bit(Mode mode) {
        return static_cast<std::uint8_t>(1u << static_cast<unsigned>(mode));
    }

    static int popcount(std::uint8_t mask) {
        int count = 0;
        for (int bit = 0; bit < 5; ++bit) {
            if ((mask & static_cast<std::uint8_t>(1u << bit)) != 0) {
                ++count;
            }
        }
        return count;
    }

    static double population_stddev(const double* values, int count) {
        if (values == nullptr || count <= 0) {
            return 0.0;
        }
        double mean = 0.0;
        for (int index = 0; index < count; ++index) {
            mean += values[index];
        }
        mean /= static_cast<double>(count);
        double accum = 0.0;
        for (int index = 0; index < count; ++index) {
            const double delta = values[index] - mean;
            accum += delta * delta;
        }
        return std::sqrt(accum / static_cast<double>(count));
    }

    FdirRules() {
        this->clear_monitor();
        this->clear_manager();
    }

    void set_thresholds(const Thresholds& thresholds) {
        this->m_thresholds = thresholds;
        if (this->m_thresholds.overspeed_samples < 1) {
            this->m_thresholds.overspeed_samples = 1;
        }
        if (this->m_thresholds.soc_samples < 1) {
            this->m_thresholds.soc_samples = 1;
        }
        if (this->m_thresholds.temp_crit_samples < 1) {
            this->m_thresholds.temp_crit_samples = 1;
        }
        if (this->m_thresholds.temp_warn_samples < 1) {
            this->m_thresholds.temp_warn_samples = 1;
        }
        if (this->m_thresholds.stddev_samples < 2) {
            this->m_thresholds.stddev_samples = 2;
        }
        if (this->m_thresholds.stddev_samples > STDDEV_CAPACITY) {
            this->m_thresholds.stddev_samples = STDDEV_CAPACITY;
        }
    }

    const Thresholds& thresholds() const { return this->m_thresholds; }

    Mode mode() const { return this->m_mode; }

    std::uint8_t wheel_index() const { return this->m_wheel; }

    // Monitor half. mode_now is the mode already commanded, used only for dump-rise.
    // Invalid input does not move counters. 10 Hz predicates count one call per sample.
    // SOC persistence counts once per SOC_PERIOD_NS of sim time (1 Hz).
    FaultRecord observe(const Sample& sample, Mode mode_now) {
        if (!sample.valid) {
            return this->frozen_invalid();
        }

        this->m_have_sample = true;
        this->m_frozen = sample;
        this->push_speeds(sample);
        this->note_overspeed(sample);
        this->note_temperature(sample);
        this->note_soc(sample);
        this->note_dump_rise(sample, mode_now);

        FaultRecord record = this->blank_record(sample, true);
        this->fill_stddev(record);
        this->choose_predicate(record, mode_now);
        return record;
    }

    // Manager half. One warning episode, then the mode stays until the fault clears.
    // CRITICAL applies default_mode on this call and does not take a proposal.
    // WARNING with two or more eligible modes waits PROPOSAL_WINDOW_NS, then default_mode.
    Decision decide(const FaultRecord& record, const Proposal* proposal) {
        if (!record.sensor_valid) {
            Decision held = this->snapshot(Disposition::HOLD_INVALID);
            held.sensor_valid = false;
            held.temp_warn_event = false;
            return held;
        }

        const Proposal* usable = proposal;
        if (proposal != nullptr) {
            this->m_last_proposal = proposal->action;
            if (this->proposal_is_stale(proposal->sim_time_ns, record.sim_time_ns)) {
                usable = nullptr;
            }
        }

        this->m_sim_time = record.sim_time_ns;
        this->m_severity = record.severity;
        this->m_eligible = record.eligible;
        this->m_default = record.default_mode;

        if (record.severity == Severity::CRITICAL) {
            this->cancel_episode();
            this->m_quiet_armed = false;
            this->apply_mode(record.default_mode, record.wheel_index);
            const Disposition disposition = (proposal != nullptr) ? Disposition::REJECT : Disposition::NONE;
            return this->snapshot(disposition, record.temp_warn_event);
        }

        if (this->m_mode == Mode::SAFE || this->m_mode == Mode::ISOLATE_RW) {
            const Disposition disposition = (proposal != nullptr) ? Disposition::REJECT : Disposition::NONE;
            return this->snapshot(disposition, record.temp_warn_event);
        }

        if (record.severity == Severity::WARNING && popcount(record.eligible) >= 1) {
            return this->decide_warning(record, proposal, usable);
        }

        return this->decide_quiet(record, proposal);
    }

    Decision tick(const Sample& sample, const Proposal* proposal) {
        const FaultRecord record = this->observe(sample, this->m_mode);
        bool filtered_out = false;
        const Proposal* usable = nullptr;
        if (proposal != nullptr) {
            const FilterResult result = this->m_filter.screen(proposal->opcode, proposal->sim_time_ns, sample.sim_time_ns,
                                                              proposal->wall_time_ns, proposal->wheel_index,
                                                              proposal->confidence);
            if (result == FilterResult::PASS) {
                usable = proposal;
            } else {
                filtered_out = true;
            }
        }
        Decision decision = this->decide(record, usable);
        if (filtered_out && decision.disposition == Disposition::NONE) {
            decision.disposition = Disposition::REJECT;
        }
        if (!record.sensor_valid) {
            decision.disposition = Disposition::HOLD_INVALID;
            decision.sensor_valid = false;
        }
        return decision;
    }

    void clear() {
        this->clear_monitor();
        this->clear_manager();
    }

    void clear_monitor() {
        this->m_over_count = 0;
        this->m_over_wheel = 0;
        this->m_temp_crit = 0;
        this->m_temp_warn = 0;
        this->m_soc_crit = 0;
        this->m_soc_warn = 0;
        this->m_soc_crit_armed = false;
        this->m_soc_warn_armed = false;
        this->m_soc_crit_anchor = 0;
        this->m_soc_warn_anchor = 0;
        this->m_hist_count = 0;
        this->m_have_peak = false;
        this->m_prev_peak = 0.0;
        this->m_have_prev_sim = false;
        this->m_prev_sim = 0;
        this->m_rise_accum = 0;
        this->m_have_sample = false;
        this->m_frozen = Sample{};
        for (int wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
            this->m_frozen.wheel_speed_rad_s[wheel] = 0.0;
            for (int sample = 0; sample < STDDEV_CAPACITY; ++sample) {
                this->m_hist[wheel][sample] = 0.0;
            }
        }
    }

    void clear_manager() {
        this->m_mode = Mode::NOMINAL;
        this->m_wheel = 0;
        this->m_severity = Severity::NONE;
        this->m_eligible = 0;
        this->m_default = Mode::NOMINAL;
        this->m_window = false;
        this->m_window_start = 0;
        this->m_episode = false;
        this->m_episode_settled = false;
        this->m_episode_mask = 0;
        this->m_quiet_armed = false;
        this->m_quiet_since = 0;
        this->m_sim_time = 0;
        this->m_last_proposal = Mode::NOMINAL;
    }

    FilterResult screen(const char* opcode, std::uint64_t proposal_sim_ns, std::uint64_t now_sim_ns,
                        std::uint64_t wall_time_ns, std::uint8_t wheel_index, float confidence) {
        return this->m_filter.screen(opcode, proposal_sim_ns, now_sim_ns, wall_time_ns, wheel_index, confidence);
    }

    class ProposalFilter {
      public:
        ProposalFilter() : m_have(false), m_last_wall(0) {}

        FilterResult screen(const char* opcode, std::uint64_t proposal_sim_ns, std::uint64_t now_sim_ns,
                            std::uint64_t wall_time_ns, std::uint8_t wheel_index, float confidence) {
            if (opcode != nullptr && std::strcmp(opcode, OPCODE_CLEAR) == 0) {
                return FilterResult::DROP_CLEAR;
            }
            if (opcode == nullptr || std::strcmp(opcode, OPCODE_PROPOSE) != 0) {
                return FilterResult::DROP_OPCODE;
            }
            if (wheel_index >= static_cast<std::uint8_t>(WHEEL_COUNT) || !(confidence >= 0.0f && confidence <= 1.0f)) {
                return FilterResult::DROP_ARGS;
            }
            const std::uint64_t separation = (now_sim_ns > proposal_sim_ns) ? (now_sim_ns - proposal_sim_ns)
                                                                            : (proposal_sim_ns - now_sim_ns);
            if (separation > PROPOSAL_STALE_NS) {
                return FilterResult::DROP_STALE;
            }
            if (this->m_have) {
                if (wall_time_ns < this->m_last_wall ||
                    (wall_time_ns - this->m_last_wall) < PROPOSAL_WINDOW_NS) {
                    return FilterResult::DROP_RATE;
                }
            }
            this->m_have = true;
            this->m_last_wall = wall_time_ns;
            return FilterResult::PASS;
        }

      private:
        bool m_have;
        std::uint64_t m_last_wall;
    };

  private:
    struct Pick {
        int rank;
        Severity severity;
        std::uint8_t eligible;
        Mode default_mode;
        std::uint8_t wheel_index;
        bool quiet;
        bool temp_warn_event;
    };

    static Pick none_pick() {
        Pick pick;
        pick.rank = 0;
        pick.severity = Severity::NONE;
        pick.eligible = 0;
        pick.default_mode = Mode::NOMINAL;
        pick.wheel_index = 0;
        pick.quiet = true;
        pick.temp_warn_event = false;
        return pick;
    }

    void push_speeds(const Sample& sample) {
        const int window = this->m_thresholds.stddev_samples;
        if (this->m_hist_count < window) {
            for (int wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
                this->m_hist[wheel][this->m_hist_count] = sample.wheel_speed_rad_s[wheel];
            }
            this->m_hist_count += 1;
            return;
        }
        for (int wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
            for (int index = 1; index < window; ++index) {
                this->m_hist[wheel][index - 1] = this->m_hist[wheel][index];
            }
            this->m_hist[wheel][window - 1] = sample.wheel_speed_rad_s[wheel];
        }
    }

    void note_overspeed(const Sample& sample) {
        const double limit = this->m_thresholds.overspeed_fraction * this->m_thresholds.wheel_max_rad_s;
        bool over = false;
        std::uint8_t wheel_index = 0;
        for (int wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
            if (std::fabs(sample.wheel_speed_rad_s[wheel]) > limit) {
                over = true;
                wheel_index = static_cast<std::uint8_t>(wheel);
                break;
            }
        }
        if (!over) {
            this->m_over_count = 0;
            return;
        }
        if (this->m_over_count < this->m_thresholds.overspeed_samples) {
            this->m_over_count += 1;
        }
        this->m_over_wheel = wheel_index;
    }

    void note_temperature(const Sample& sample) {
        if (sample.temp_c > this->m_thresholds.temp_crit_c) {
            if (this->m_temp_crit < this->m_thresholds.temp_crit_samples) {
                this->m_temp_crit += 1;
            }
            this->m_temp_warn = 0;
            return;
        }
        this->m_temp_crit = 0;
        if (sample.temp_c > this->m_thresholds.temp_warn_c && sample.temp_c < this->m_thresholds.temp_crit_c) {
            if (this->m_temp_warn < this->m_thresholds.temp_warn_samples) {
                this->m_temp_warn += 1;
            }
            return;
        }
        this->m_temp_warn = 0;
    }

    static void note_period(bool active, int limit, int& count, bool& armed, std::uint64_t& anchor,
                            std::uint64_t sim_time_ns) {
        if (!active) {
            count = 0;
            armed = false;
            return;
        }
        if (!armed) {
            armed = true;
            count = 1;
            anchor = sim_time_ns;
            return;
        }
        if (sim_time_ns < anchor || count >= limit) {
            if (count > limit) {
                count = limit;
            }
            return;
        }
        const std::uint64_t elapsed = sim_time_ns - anchor;
        if (elapsed < SOC_PERIOD_NS) {
            return;
        }
        std::uint64_t steps = elapsed / SOC_PERIOD_NS;
        if (steps > static_cast<std::uint64_t>(limit)) {
            steps = static_cast<std::uint64_t>(limit);
        }
        count += static_cast<int>(steps);
        anchor += steps * SOC_PERIOD_NS;
        if (count > limit) {
            count = limit;
        }
    }

    void note_soc(const Sample& sample) {
        const bool critical = sample.soc < this->m_thresholds.soc_crit;
        const bool warning = !critical && sample.soc < this->m_thresholds.soc_warn;
        note_period(critical, this->m_thresholds.soc_samples, this->m_soc_crit, this->m_soc_crit_armed,
                    this->m_soc_crit_anchor, sample.sim_time_ns);
        note_period(warning, this->m_thresholds.soc_samples, this->m_soc_warn, this->m_soc_warn_armed,
                    this->m_soc_warn_anchor, sample.sim_time_ns);
    }

    void note_dump_rise(const Sample& sample, Mode mode_now) {
        double peak = 0.0;
        for (int wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
            const double speed = std::fabs(sample.wheel_speed_rad_s[wheel]);
            if (speed > peak) {
                peak = speed;
            }
        }
        if (mode_now == Mode::MOMENTUM_DUMP && this->m_have_peak && this->m_have_prev_sim &&
            sample.sim_time_ns >= this->m_prev_sim) {
            if (peak > this->m_prev_peak) {
                this->m_rise_accum += sample.sim_time_ns - this->m_prev_sim;
            } else {
                this->m_rise_accum = 0;
            }
        } else if (mode_now != Mode::MOMENTUM_DUMP) {
            this->m_rise_accum = 0;
        }
        this->m_prev_peak = peak;
        this->m_have_peak = true;
        this->m_prev_sim = sample.sim_time_ns;
        this->m_have_prev_sim = true;
    }

    void fill_stddev(FaultRecord& record) const {
        const int window = this->m_thresholds.stddev_samples;
        for (int wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
            record.wheel_stddev[wheel] = 0.0;
        }
        if (this->m_hist_count < window) {
            return;
        }
        for (int wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
            record.wheel_stddev[wheel] = population_stddev(this->m_hist[wheel], window);
        }
    }

    void choose_predicate(FaultRecord& record, Mode mode_now) {
        Pick best = none_pick();
        if (this->m_temp_crit >= this->m_thresholds.temp_crit_samples) {
            consider(best, 100, Severity::CRITICAL, mode_bit(Mode::SAFE), Mode::SAFE, 0);
        }
        if (this->m_soc_crit >= this->m_thresholds.soc_samples) {
            consider(best, 90, Severity::CRITICAL, mode_bit(Mode::SAFE), Mode::SAFE, 0);
        }
        if (mode_now == Mode::MOMENTUM_DUMP && this->m_rise_accum >= this->m_thresholds.dump_rise_ns) {
            consider(best, 80, Severity::CRITICAL, mode_bit(Mode::SAFE), Mode::SAFE, this->m_over_wheel);
        }
        if (this->m_over_count >= this->m_thresholds.overspeed_samples) {
            consider(best, 70, Severity::WARNING, static_cast<std::uint8_t>(mode_bit(Mode::MOMENTUM_DUMP) | mode_bit(Mode::SAFE)),
                     Mode::MOMENTUM_DUMP, this->m_over_wheel);
        }
        std::uint8_t noisy = 0;
        int high = 0;
        int quiet = 0;
        if (this->m_hist_count >= this->m_thresholds.stddev_samples) {
            for (int wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
                if (record.wheel_stddev[wheel] >= this->m_thresholds.stddev_high) {
                    ++high;
                    noisy = static_cast<std::uint8_t>(wheel);
                } else if (record.wheel_stddev[wheel] <= this->m_thresholds.stddev_quiet) {
                    ++quiet;
                }
            }
        }
        if (high == 1 && quiet == WHEEL_COUNT - 1) {
            consider(best, 60, Severity::WARNING,
                     static_cast<std::uint8_t>(mode_bit(Mode::ISOLATE_RW) | mode_bit(Mode::NOMINAL)), Mode::ISOLATE_RW,
                     noisy);
        }
        if (this->m_soc_warn >= this->m_thresholds.soc_samples) {
            consider(best, 50, Severity::WARNING,
                     static_cast<std::uint8_t>(mode_bit(Mode::LOAD_SHED) | mode_bit(Mode::NOMINAL)), Mode::LOAD_SHED, 0);
        }
        if (best.rank == 0 && this->m_temp_warn >= this->m_thresholds.temp_warn_samples) {
            best.temp_warn_event = true;
            best.quiet = false;
        }
        record.severity = best.severity;
        record.eligible = best.eligible;
        record.default_mode = best.default_mode;
        record.wheel_index = best.wheel_index;
        record.quiet = best.quiet && best.rank == 0;
        record.temp_warn_event = best.temp_warn_event;
    }

    static void consider(Pick& best, int rank, Severity severity, std::uint8_t eligible, Mode default_mode,
                         std::uint8_t wheel_index) {
        if (rank <= best.rank) {
            return;
        }
        best.rank = rank;
        best.severity = severity;
        best.eligible = eligible;
        best.default_mode = default_mode;
        best.wheel_index = wheel_index;
        best.quiet = false;
        best.temp_warn_event = false;
    }

    Decision decide_warning(const FaultRecord& record, const Proposal* proposal, const Proposal* usable) {
        const int members = popcount(record.eligible);
        if (members == 1) {
            this->cancel_episode();
            Mode only = record.default_mode;
            for (int bit = 0; bit < 5; ++bit) {
                if ((record.eligible & static_cast<std::uint8_t>(1u << bit)) != 0) {
                    only = static_cast<Mode>(bit);
                }
            }
            this->apply_mode(only, record.wheel_index);
            Disposition disposition = Disposition::NONE;
            if (proposal != nullptr) {
                disposition = (usable != nullptr && usable->action == only)
                                  ? ((only == record.default_mode) ? Disposition::AGREE : Disposition::DISAGREE)
                                  : Disposition::REJECT;
            }
            return this->snapshot(disposition, record.temp_warn_event);
        }

        const bool same_episode = this->m_episode && this->m_episode_settled && this->m_episode_mask == record.eligible;
        if (same_episode) {
            const Disposition disposition = (proposal != nullptr) ? Disposition::LATE : Disposition::NONE;
            return this->snapshot(disposition, record.temp_warn_event);
        }

        if (!this->m_episode || this->m_episode_mask != record.eligible || !this->m_window) {
            this->m_window = true;
            this->m_window_start = record.sim_time_ns;
            this->m_episode = true;
            this->m_episode_settled = false;
            this->m_episode_mask = record.eligible;
        }

        std::uint64_t elapsed = 0;
        if (record.sim_time_ns >= this->m_window_start) {
            elapsed = record.sim_time_ns - this->m_window_start;
        }

        Disposition disposition = Disposition::NONE;
        bool consumed = false;
        if (proposal != nullptr) {
            if (usable == nullptr) {
                disposition = Disposition::REJECT;
            } else if ((record.eligible & mode_bit(usable->action)) == 0) {
                disposition = Disposition::REJECT;
            } else if (elapsed > PROPOSAL_WINDOW_NS) {
                disposition = Disposition::LATE;
            } else {
                this->apply_mode(usable->action, record.wheel_index);
                disposition = (usable->action == record.default_mode) ? Disposition::AGREE : Disposition::DISAGREE;
                consumed = true;
                this->m_window = false;
                this->m_episode_settled = true;
            }
        }
        if (!consumed && elapsed >= PROPOSAL_WINDOW_NS) {
            this->apply_mode(record.default_mode, record.wheel_index);
            this->m_window = false;
            this->m_episode_settled = true;
        }
        return this->snapshot(disposition, record.temp_warn_event);
    }

    Decision decide_quiet(const FaultRecord& record, const Proposal* proposal) {
        this->cancel_episode();
        if (this->m_mode == Mode::MOMENTUM_DUMP || this->m_mode == Mode::LOAD_SHED) {
            if (!record.quiet) {
                this->m_quiet_armed = false;
            } else if (!this->m_quiet_armed) {
                this->m_quiet_armed = true;
                this->m_quiet_since = record.sim_time_ns;
            } else if (record.sim_time_ns >= this->m_quiet_since &&
                       (record.sim_time_ns - this->m_quiet_since) >= this->m_thresholds.quiet_dwell_ns) {
                this->apply_mode(Mode::NOMINAL, 0);
                this->m_quiet_armed = false;
            }
        } else {
            this->m_quiet_armed = false;
        }
        const Disposition disposition = (proposal != nullptr) ? Disposition::REJECT : Disposition::NONE;
        return this->snapshot(disposition, record.temp_warn_event);
    }

    void apply_mode(Mode mode, std::uint8_t wheel_index) {
        this->m_mode = mode;
        this->m_wheel = wheel_index;
    }

    void cancel_episode() {
        this->m_window = false;
        this->m_episode = false;
        this->m_episode_settled = false;
        this->m_episode_mask = 0;
    }

    bool proposal_is_stale(std::uint64_t proposal_sim_ns, std::uint64_t now_sim_ns) const {
        const std::uint64_t separation = (now_sim_ns > proposal_sim_ns) ? (now_sim_ns - proposal_sim_ns)
                                                                        : (proposal_sim_ns - now_sim_ns);
        return separation > PROPOSAL_STALE_NS;
    }

    FaultRecord blank_record(const Sample& sample, bool sensor_valid) const {
        FaultRecord record{};
        record.sim_time_ns = sample.sim_time_ns;
        record.eligible = 0;
        record.default_mode = Mode::NOMINAL;
        record.severity = Severity::NONE;
        record.wheel_index = 0;
        record.sensor_valid = sensor_valid;
        record.quiet = false;
        record.temp_warn_event = false;
        record.soc = sample.soc;
        record.solar_scale = sample.solar_scale;
        record.temp_c = sample.temp_c;
        for (int wheel = 0; wheel < WHEEL_COUNT; ++wheel) {
            record.wheel_speed_rad_s[wheel] = sample.wheel_speed_rad_s[wheel];
            record.wheel_stddev[wheel] = 0.0;
        }
        return record;
    }

    FaultRecord frozen_invalid() const {
        Sample sample = this->m_frozen;
        sample.valid = false;
        if (!this->m_have_sample) {
            sample.sim_time_ns = 0;
        }
        FaultRecord record = this->blank_record(sample, false);
        record.quiet = false;
        return record;
    }

    Decision snapshot(Disposition disposition, bool temp_warn_event = false) const {
        Decision decision;
        decision.mode = this->m_mode;
        decision.severity = this->m_severity;
        decision.eligible = this->m_eligible;
        decision.default_mode = this->m_default;
        decision.disposition = disposition;
        decision.wheel_index = this->m_wheel;
        decision.sensor_valid = true;
        decision.temp_warn_event = temp_warn_event;
        decision.sim_time_ns = this->m_sim_time;
        decision.last_proposal = this->m_last_proposal;
        return decision;
    }

    Thresholds m_thresholds;
    ProposalFilter m_filter;
    Sample m_frozen;
    bool m_have_sample;
    int m_over_count;
    std::uint8_t m_over_wheel;
    int m_temp_crit;
    int m_temp_warn;
    int m_soc_crit;
    int m_soc_warn;
    bool m_soc_crit_armed;
    bool m_soc_warn_armed;
    std::uint64_t m_soc_crit_anchor;
    std::uint64_t m_soc_warn_anchor;
    double m_hist[WHEEL_COUNT][STDDEV_CAPACITY];
    int m_hist_count;
    bool m_have_peak;
    double m_prev_peak;
    bool m_have_prev_sim;
    std::uint64_t m_prev_sim;
    std::uint64_t m_rise_accum;
    Mode m_mode;
    std::uint8_t m_wheel;
    Severity m_severity;
    std::uint8_t m_eligible;
    Mode m_default;
    bool m_window;
    std::uint64_t m_window_start;
    bool m_episode;
    bool m_episode_settled;
    std::uint8_t m_episode_mask;
    bool m_quiet_armed;
    std::uint64_t m_quiet_since;
    std::uint64_t m_sim_time;
    Mode m_last_proposal;
};

}  // namespace Orbiedge

#endif
