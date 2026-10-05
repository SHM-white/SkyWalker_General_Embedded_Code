#pragma once
#include <cstdint>

namespace skywalker::samples::control {
enum class DiagnosticScenario : unsigned {
    None, InputPause, ExecutionPause, StatusPause, HeadPause, VisionPause,
    PermissionPause, MeasurementPause, InvalidProtocol
};
struct DiagnosticState {
    bool input_paused = false, execution_paused = false, status_paused = false;
    bool head_paused = false, vision_paused = false, permission_paused = false, measurement_paused = false;
    bool invalid_protocol = false;
};
#ifdef CONFIG_SAMPLE_DIAGNOSTIC_SCENARIO
inline constexpr auto diagnostic_scenario = static_cast<DiagnosticScenario>(CONFIG_SAMPLE_DIAGNOSTIC_SCENARIO);
#else
inline constexpr auto diagnostic_scenario = DiagnosticScenario::None;
#endif

// Independent management owner: never pause this timer with the injected
// producer. A run gets at most one exercise per boot; RC stop cancels it.
class SampleDiagnostics {
public:
    explicit SampleDiagnostics(DiagnosticScenario scenario = diagnostic_scenario) : scenario_(scenario) {}
    DiagnosticState update(std::uint64_t now_ms, bool active, bool safe_or_offline = false) {
        DiagnosticState state{};
        if (scenario_ == DiagnosticScenario::None || done_) return state;
        if (safe_or_offline) {
            if (tracking_ || triggered_) done_ = true;
            tracking_ = triggered_ = false;
            return state;
        }
        if (!triggered_) {
            if (!active) { tracking_ = false; return state; }
            if (!tracking_) { stable_since_ = now_ms; tracking_ = true; }
            if (now_ms < stable_since_ || now_ms - stable_since_ < 3000) return state;
            triggered_ = true;
            triggered_at_ = now_ms;
        }
        if (now_ms < triggered_at_ || now_ms - triggered_at_ >= 1500) { done_ = true; return state; }
        switch (scenario_) {
        case DiagnosticScenario::InputPause: state.input_paused = true; break;
        case DiagnosticScenario::ExecutionPause: state.execution_paused = true; break;
        case DiagnosticScenario::StatusPause: state.status_paused = true; break;
        case DiagnosticScenario::HeadPause: state.head_paused = true; break;
        case DiagnosticScenario::VisionPause: state.vision_paused = true; break;
        case DiagnosticScenario::PermissionPause: state.permission_paused = true; break;
        case DiagnosticScenario::MeasurementPause: state.measurement_paused = true; break;
        case DiagnosticScenario::InvalidProtocol: state.invalid_protocol = true; break;
        default: break;
        }
        return state;
    }
private:
    DiagnosticScenario scenario_;
    std::uint64_t stable_since_ = 0, triggered_at_ = 0;
    bool tracking_ = false, triggered_ = false, done_ = false;
};
} // namespace skywalker::samples::control
