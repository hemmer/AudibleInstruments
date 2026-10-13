#include "plugin.hpp"
#include "Beads/BeadsEngine.hpp"
#include "Beads/ControlLayer.hpp"
#include "Beads/PanelLayout.hpp"
#include "beads/pot_controller.h"

#include <atomic>
#include <memory>

static constexpr float kMenuGainValues[] = {0.f, .25f, .5f, .75f, 1.f};
static constexpr const char* kMenuGainLabels[] = {"Minimum", "25%", "50%", "75%", "Maximum"};
static constexpr float kMenuAmountValues[] = {0.f, .25f, .5f, .75f, 1.f};
static constexpr const char* kMenuAmountLabels[] = {"0%", "25%", "50%", "75%", "100%"};

struct Beads : Module {
    enum ParamIds {
        MEDIA_PARAM, FREEZE_PARAM, SEED_PARAM, ASSIGN_PARAM,
        TIME_PARAM, SIZE_PARAM, SHAPE_PARAM, PITCH_PARAM,
        TIME_AMOUNT_PARAM, SIZE_AMOUNT_PARAM, SHAPE_AMOUNT_PARAM, PITCH_AMOUNT_PARAM,
        DENSITY_PARAM, FEEDBACK_PARAM, DRY_WET_PARAM, REVERB_PARAM,
        INPUT_GAIN_PARAM, FEEDBACK_CV_AMOUNT_PARAM, DRY_WET_CV_AMOUNT_PARAM,
        REVERB_CV_AMOUNT_PARAM, NUM_PARAMS
    };
    enum InputIds {
        TIME_CV_INPUT, SIZE_CV_INPUT, SHAPE_CV_INPUT, PITCH_CV_INPUT,
        DENSITY_CV_INPUT, ASSIGN_CV_INPUT, SEED_INPUT, FREEZE_INPUT,
        AUDIO_L_INPUT, AUDIO_R_INPUT, NUM_INPUTS
    };
    enum OutputIds { AUDIO_L_OUTPUT, AUDIO_R_OUTPUT, NUM_OUTPUTS };
    enum LightIds {
        MEDIA_R_LIGHT, MEDIA_G_LIGHT, MEDIA_B_LIGHT,
        FREEZE_LIGHT, SEED_LIGHT, ASSIGN_1_LIGHT, ASSIGN_2_LIGHT,
        ASSIGN_3_LIGHT, VU_LIGHT, ERROR_LIGHT, NUM_LIGHTS
    };

    std::unique_ptr<audible_beads::BeadsEngine> engine;
    bool down[4]{};
    uint64_t held[4]{};
    uint64_t sampleIndex = 0;
    double seedIndicatorPhase = 0.;
    uint64_t restartGainSamples = 0;
    bool seedLocked = true;
    bool freezeLatch = false;
    bool mediaLongHandled = false;
    bool comboConsumed = false;
    bool previousButton[4]{};
    bool seedGateLight = false, freezeGateLight = false;
    bool gainManual = false;
    bool gateOutput = false;
    int media = audible_beads::ControlLayer::MEDIA_DIGITAL;
    int assignment = 0; // 0..2 direct destination, 3 custom amounts.
    std::array<float, 3> assignmentAmounts{{1.f, 0.f, .5f}};
    std::atomic<int> menuMedia{3}, menuSeedLocked{1}, menuGainManual{0};
    std::atomic<int> menuAssignment{0}, menuGateOutput{0};
    std::atomic<int> menuFreezeLatch{0}, resetRequested{0};
    std::atomic<float> menuManualGain{1.f};
    std::atomic<int> editView{0}; // UI-only: normal, input gain, CV amounts.
    std::atomic<bool> statusSynth{true};
    std::atomic<bool> settingsSyncRequested{false};
    std::array<std::atomic<float>, 4> lastSettingParams{{1.f, 1.f, 0.f, .5f}};
    std::array<std::atomic<float>, 3> menuAssignmentAmounts{{1.f, 0.f, .5f}};
    float manualGain = 1.f;
    beads::PotController mixPots[3];
    double uiPhase = 0., controlPhase = 0.;
    bool assignmentEdited[3]{};
    bool gainEdited = false;
    audible_beads::ControlLayer::Smoother smooth[18];
    float grainBase[4]{.25f, .5f, .5f, 0.f};
    float grainAmount[4]{};
    float grainCv[4]{};
    float densityBase = .625f, densityCv = 0.f, assignCv = 0.f;
    float timeHysteresis = 0.f;

    Beads() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
        configButton(MEDIA_PARAM, "Media"); configButton(FREEZE_PARAM, "Freeze");
        configButton(SEED_PARAM, "Seed"); configButton(ASSIGN_PARAM, "CV assignment");
        for (int id : {MEDIA_PARAM, FREEZE_PARAM, SEED_PARAM, ASSIGN_PARAM})
            getParamQuantity(id)->randomizeEnabled = false;
        configParam(TIME_PARAM, 0.f, 1.f, .25f, "Grain time");
        configParam(SIZE_PARAM, 0.f, 1.f, .5f, "Grain size");
        configParam(SHAPE_PARAM, 0.f, 1.f, .5f, "Grain shape");
        configParam(PITCH_PARAM, 0.f, 1.f, .5f, "Grain pitch");
        configParam(TIME_AMOUNT_PARAM, -1.f, 1.f, 0.f, "Time CV / randomness amount");
        configParam(SIZE_AMOUNT_PARAM, -1.f, 1.f, 0.f, "Size CV / randomness amount");
        configParam(SHAPE_AMOUNT_PARAM, -1.f, 1.f, 0.f, "Shape CV / randomness amount");
        configParam(PITCH_AMOUNT_PARAM, -1.f, 1.f, 0.f, "Pitch CV / randomness amount");
        configParam(DENSITY_PARAM, -1.f, 1.f, .25f, "Grain density");
        getParamQuantity(DENSITY_PARAM)->description = "Negative: regular grains / clock division.\nZero: idle center.\nAt zero, unlatched: one grain per trigger.\nPositive: random grains / clock probability.";
        configParam(FEEDBACK_PARAM, 0.f, 1.f, 0.f, "Feedback");
        configParam(DRY_WET_PARAM, 0.f, 1.f, .5f, "Dry/wet mix");
        configParam(REVERB_PARAM, 0.f, 1.f, 0.f, "Reverb");
        configParam(INPUT_GAIN_PARAM, 0.f, 1.f, 1.f, "Manual input gain setting", "%", 0.f, 100.f);
        configParam(FEEDBACK_CV_AMOUNT_PARAM, 0.f, 1.f, 1.f, "Feedback CV amount", "%", 0.f, 100.f);
        configParam(DRY_WET_CV_AMOUNT_PARAM, 0.f, 1.f, 0.f, "Dry/wet CV amount", "%", 0.f, 100.f);
        configParam(REVERB_CV_AMOUNT_PARAM, 0.f, 1.f, .5f, "Reverb CV amount", "%", 0.f, 100.f);
        for (int id : {INPUT_GAIN_PARAM, FEEDBACK_CV_AMOUNT_PARAM, DRY_WET_CV_AMOUNT_PARAM, REVERB_CV_AMOUNT_PARAM})
            getParamQuantity(id)->randomizeEnabled = false;
        configInput(TIME_CV_INPUT, "Time CV"); configInput(SIZE_CV_INPUT, "Size CV");
        configInput(SHAPE_CV_INPUT, "Shape CV"); configInput(PITCH_CV_INPUT, "Pitch CV");
        configInput(DENSITY_CV_INPUT, "Density CV"); configInput(ASSIGN_CV_INPUT, "Assignable mix CV");
        configInput(SEED_INPUT, "Seed gate / clock"); configInput(FREEZE_INPUT, "Freeze gate");
        configInput(AUDIO_L_INPUT, "Audio left")->description = "Uses the first cable channel.\nA single input is treated as mono.";
        configInput(AUDIO_R_INPUT, "Audio right")->description = "Uses the first cable channel.\nA single input is treated as mono.";
        configOutput(AUDIO_L_OUTPUT, "Audio left / mono sum"); configOutput(AUDIO_R_OUTPUT, "Audio right / optional grain gate");
        configBypass(AUDIO_L_INPUT, AUDIO_L_OUTPUT);
        configBypass(AUDIO_R_INPUT, AUDIO_R_OUTPUT);
        getParamQuantity(PITCH_PARAM)->description = "Pitch pot: stepped semitone plateaus.\nPitch CV: continuous at 1 V/octave.";
        getParamQuantity(TIME_AMOUNT_PARAM)->description = "Without CV:\n  CW: uniform randomization.\n  CCW: peaky randomization.\nWith CV:\n  CW: attenuates the signal.\n  CCW: positive CV controls randomization.";
        getParamQuantity(SIZE_AMOUNT_PARAM)->description = getParamQuantity(TIME_AMOUNT_PARAM)->description;
        getParamQuantity(SHAPE_AMOUNT_PARAM)->description = getParamQuantity(TIME_AMOUNT_PARAM)->description;
        getParamQuantity(PITCH_AMOUNT_PARAM)->description = getParamQuantity(TIME_AMOUNT_PARAM)->description;
        getParamQuantity(SEED_PARAM)->description = "Momentary seed / clock.\nHold 4 seconds and release to latch.\nSEED + FREEZE also locks the seed.";
        getParamQuantity(MEDIA_PARAM)->description = "Short press: change media.\nHold 1.5 seconds, then release:\nrestart automatic gain adjustment.";
        engine = std::make_unique<audible_beads::BeadsEngine>();
        for (int i = 0; i < 3; ++i) {
            mixPots[i].Init(1.f, 0.f);
            const int ids[3] = {FEEDBACK_PARAM, DRY_WET_PARAM, REVERB_PARAM};
            mixPots[i].ProcessControlRate(params[ids[i]].getValue());
        }
        onReset();
    }

    void onReset() override {
        menuMedia.store(3); menuSeedLocked.store(1); menuGainManual.store(0);
        menuAssignment.store(0); menuGateOutput.store(0); menuManualGain.store(1.f);
        menuAssignmentAmounts[0].store(1.f); menuAssignmentAmounts[1].store(0.f); menuAssignmentAmounts[2].store(.5f);
        menuFreezeLatch.store(0); resetRequested.store(1);
        params[INPUT_GAIN_PARAM].setValue(1.f);
        params[FEEDBACK_CV_AMOUNT_PARAM].setValue(1.f);
        params[DRY_WET_CV_AMOUNT_PARAM].setValue(0.f);
        params[REVERB_CV_AMOUNT_PARAM].setValue(.5f);
        editView.store(0); settingsSyncRequested.store(true);
    }

    float savedSettingValue(int i) {
        const int ids[4] = {INPUT_GAIN_PARAM, FEEDBACK_CV_AMOUNT_PARAM, DRY_WET_CV_AMOUNT_PARAM, REVERB_CV_AMOUNT_PARAM};
        const float value = audible_beads::ControlLayer::clamp01(getParamQuantity(ids[i])->getValue());
        // Preserve knob edits made while Rack's engine is paused, as well as
        // context-menu commands not yet mirrored into their parameters.
        if (value != lastSettingParams[i].load()) return value;
        return i == 0 ? menuManualGain.load() : menuAssignmentAmounts[i - 1].load();
    }

    json_t* dataToJson() override {
        json_t* root = json_object();
        json_object_set_new(root, "beadsVersion", json_integer(2));
        json_object_set_new(root, "media", json_integer(std::clamp(menuMedia.load(), 0, 3)));
        json_object_set_new(root, "seedLocked", json_boolean(menuSeedLocked.load() != 0));
        json_object_set_new(root, "freezeLatch", json_boolean(menuFreezeLatch.load() != 0));
        json_object_set_new(root, "gainManual", json_boolean(menuGainManual.load() != 0 || getParamQuantity(INPUT_GAIN_PARAM)->getValue() != lastSettingParams[0].load()));
        json_object_set_new(root, "manualGain", json_real(savedSettingValue(0)));
        json_object_set_new(root, "assignment", json_integer((getParamQuantity(FEEDBACK_CV_AMOUNT_PARAM)->getValue() != lastSettingParams[1].load() || getParamQuantity(DRY_WET_CV_AMOUNT_PARAM)->getValue() != lastSettingParams[2].load() || getParamQuantity(REVERB_CV_AMOUNT_PARAM)->getValue() != lastSettingParams[3].load()) ? 3 : std::clamp(menuAssignment.load(), 0, 3)));
        json_t* amounts = json_array();
        for (int i = 1; i < 4; ++i) json_array_append_new(amounts, json_real(savedSettingValue(i)));
        json_object_set_new(root, "assignmentAmounts", amounts);
        json_object_set_new(root, "gateOutput", json_boolean(menuGateOutput.load() != 0));
        return root;
    }

    void dataFromJson(json_t* root) override {
        if (!root) return;
        // Version-less development patches use the same key layout. Explicit
        // unknown versions fall back to safe defaults.
        json_t* version = json_object_get(root, "beadsVersion");
        if (version && (!json_is_integer(version) || (json_integer_value(version) != 1 && json_integer_value(version) != 2))) {
            onReset();
            return;
        }
        auto getInt = [root](const char* key, int fallback, int low, int high) {
            json_t* v = json_object_get(root, key);
            if (!json_is_integer(v)) return fallback;
            int64_t value = json_integer_value(v);
            return int(std::clamp<int64_t>(value, low, high));
        };
        auto getBool = [root](const char* key, bool fallback) {
            json_t* v = json_object_get(root, key);
            return json_is_boolean(v) ? json_is_true(v) : fallback;
        };
        int loadedMedia = getInt("media", 3, 0, 3);
        int loadedAssignment = getInt("assignment", 0, 0, 3);
        bool loadedSeed = getBool("seedLocked", true), loadedFreeze = getBool("freezeLatch", false);
        bool loadedGain = getBool("gainManual", false), loadedGate = getBool("gateOutput", false);
        menuAssignmentAmounts[0].store(1.f); menuAssignmentAmounts[1].store(0.f);
        menuAssignmentAmounts[2].store(.5f);
        json_t* gain = json_object_get(root, "manualGain");
        float loadedGainValue = (json_is_number(gain) && std::isfinite(float(json_number_value(gain))))
            ? std::clamp(float(json_number_value(gain)), 0.f, 1.f) : 1.f;
        json_t* amounts = json_object_get(root, "assignmentAmounts");
        if (json_is_array(amounts) && json_array_size(amounts) == 3) {
            for (size_t i = 0; i < 3; ++i) {
                json_t* v = json_array_get(amounts, i);
                if (json_is_number(v) && std::isfinite(float(json_number_value(v))))
                    menuAssignmentAmounts[i].store(std::clamp(float(json_number_value(v)), 0.f, 1.f));
            }
        }
        menuMedia.store(loadedMedia); menuSeedLocked.store(loadedSeed);
        menuGainManual.store(loadedGain); menuManualGain.store(loadedGainValue);
        menuAssignment.store(loadedAssignment); menuGateOutput.store(loadedGate);
        menuFreezeLatch.store(loadedFreeze);
        params[INPUT_GAIN_PARAM].setValue(loadedGainValue);
        params[FEEDBACK_CV_AMOUNT_PARAM].setValue(menuAssignmentAmounts[0].load());
        params[DRY_WET_CV_AMOUNT_PARAM].setValue(menuAssignmentAmounts[1].load());
        params[REVERB_CV_AMOUNT_PARAM].setValue(menuAssignmentAmounts[2].load());
        settingsSyncRequested.store(true);
    }

    void fromJson(json_t* root) override {
        // Rack loads parameters before module data. Preserve the physical knob
        // position of the earlier 0..1 density control, including versionless patches.
        json_t* version = json_object_get(json_object_get(root, "data"), "beadsVersion");
        const bool legacy = !version || (json_is_integer(version) && json_integer_value(version) == 1);
        bool hasDensity = false;
        json_t* values = json_object_get(root, "params");
        size_t i; json_t* value;
        json_array_foreach(values, i, value) {
            json_t* id = json_object_get(value, "id");
            if (!id) id = json_object_get(value, "paramId");
            const int64_t paramId = json_is_integer(id) ? json_integer_value(id) : int64_t(i);
            if (paramId == DENSITY_PARAM && json_is_number(json_object_get(value, "value"))) hasDensity = true;
        }
        Module::fromJson(root);
        if (legacy && hasDensity)
            params[DENSITY_PARAM].setValue(2.f * audible_beads::ControlLayer::clamp01(params[DENSITY_PARAM].getValue()) - 1.f);
    }

    void syncSettingParams() {
        const int ids[4] = {INPUT_GAIN_PARAM, FEEDBACK_CV_AMOUNT_PARAM, DRY_WET_CV_AMOUNT_PARAM, REVERB_CV_AMOUNT_PARAM};
        const bool loaded = settingsSyncRequested.exchange(false);
        for (int i = 0; i < 4; ++i) {
            auto& setting = i == 0 ? menuManualGain : menuAssignmentAmounts[i - 1];
            float value = audible_beads::ControlLayer::clamp01(params[ids[i]].getValue());
            if (loaded) {
                // A knob can be moved before the first callback (paused Rack).
                // Loaded settings have matching parameter/mirror values.
                if (value != audible_beads::ControlLayer::clamp01(setting.load())) {
                    setting.store(value);
                    if (i == 0) menuGainManual.store(1);
                    else menuAssignment.store(3);
                }
                lastSettingParams[i] = value;
            } else if (value != lastSettingParams[i]) {
                setting.store(value);
                if (i == 0) menuGainManual.store(1);
                else menuAssignment.store(3);
            } else {
                value = audible_beads::ControlLayer::clamp01(setting.load());
                params[ids[i]].setValue(value);
            }
            lastSettingParams[i] = value;
        }
    }

    void process(const ProcessArgs& args) override {
        // Only the audio thread touches engine state. Menu actions cross threads
        // as validated scalar atomics and take effect at the next process call.
        if (resetRequested.exchange(0, std::memory_order_relaxed)) {
            engine->reset();
            for (auto& s : smooth) s = audible_beads::ControlLayer::Smoother{};
            grainBase[0] = .25f; grainBase[1] = grainBase[2] = .5f; grainBase[3] = 0.f;
            grainAmount[0] = grainAmount[1] = grainAmount[2] = grainAmount[3] = 0.f;
            grainCv[0] = grainCv[1] = grainCv[2] = grainCv[3] = 0.f;
            densityBase = .625f; densityCv = assignCv = 0.f;
            for (int i = 0; i < 3; ++i) {
                mixPots[i].Init(1.f, 0.f);
                const int ids[3] = {FEEDBACK_PARAM, DRY_WET_PARAM, REVERB_PARAM};
                mixPots[i].ProcessControlRate(params[ids[i]].getValue());
                assignmentEdited[i] = false;
            }
            gainEdited = false;
            mediaLongHandled = comboConsumed = false;
            seedGateLight = freezeGateLight = false;
            seedIndicatorPhase = 0.;
            controlPhase = uiPhase = 0.; restartGainSamples = 0; timeHysteresis = 0.f;
            for (int i = 0; i < 4; ++i) { down[i] = previousButton[i] = false; held[i] = 0; }
        }
        syncSettingParams();
        media = std::clamp(menuMedia.load(std::memory_order_relaxed), 0, 3);
        seedLocked = menuSeedLocked.load(std::memory_order_relaxed) != 0;
        gainManual = menuGainManual.load(std::memory_order_relaxed) != 0;
        assignment = menuAssignment.load(std::memory_order_relaxed);
        gateOutput = menuGateOutput.load(std::memory_order_relaxed) != 0;
        freezeLatch = menuFreezeLatch.load(std::memory_order_relaxed) != 0;
        manualGain = menuManualGain.load(std::memory_order_relaxed);
        for (int i = 0; i < 3; ++i) assignmentAmounts[i] = std::clamp(menuAssignmentAmounts[i].load(std::memory_order_relaxed), 0.f, 1.f);
        updateButtons(args);
        const bool supportedRate = engine->setHostRate(int(std::lround(args.sampleRate)));
        const int mixIds[3] = {FEEDBACK_PARAM, DRY_WET_PARAM, REVERB_PARAM};
        controlPhase += 4000.0 / std::max(double(args.sampleRate), 1.0);
        if (controlPhase >= 1.0) {
            controlPhase -= 1.0;
            const int baseIds[4] = {TIME_PARAM, SIZE_PARAM, SHAPE_PARAM, PITCH_PARAM};
            const int amountIds[4] = {TIME_AMOUNT_PARAM, SIZE_AMOUNT_PARAM, SHAPE_AMOUNT_PARAM, PITCH_AMOUNT_PARAM};
            const int cvIds[4] = {TIME_CV_INPUT, SIZE_CV_INPUT, SHAPE_CV_INPUT, PITCH_CV_INPUT};
            for (int i = 0; i < 4; ++i) {
                float pot = audible_beads::ControlLayer::clamp01(params[baseIds[i]].getValue());
                if (i == 3) pot = audible_beads::ControlLayer::pitchPot(pot);
                smooth[i].process(pot, .05f);
                if (i == 0) {
                    float error = smooth[i].value - timeHysteresis;
                    if (error > .001f) timeHysteresis = smooth[i].value - .001f;
                    else if (error < -.001f) timeHysteresis = smooth[i].value + .001f;
                    grainBase[i] = timeHysteresis;
                } else grainBase[i] = smooth[i].value;
                smooth[i + 4].process(audible_beads::ControlLayer::attenuverterPotLaw(
                    params[amountIds[i]].getValue()), .05f);
                grainAmount[i] = smooth[i + 4].value;
                float cv = inputs[cvIds[i]].isConnected()
                    ? audible_beads::ControlLayer::finite(inputs[cvIds[i]].getVoltage()) : 0.f;
                if (i < 3) {
                    smooth[12 + i].process(cv / 5.f, .5f);
                    grainCv[i] = smooth[12 + i].value;
                } else grainCv[i] = cv * 12.f;
            }
            smooth[8].process(.5f * (audible_beads::ControlLayer::attenuverter(params[DENSITY_PARAM].getValue()) + 1.f), .05f);
            densityBase = smooth[8].value;
            densityCv = inputs[DENSITY_CV_INPUT].isConnected()
                ? std::clamp(audible_beads::ControlLayer::finite(inputs[DENSITY_CV_INPUT].getVoltage()) / 5.f, -1.f, 1.f) : 0.f;
            smooth[17].process(inputs[ASSIGN_CV_INPUT].isConnected()
                ? std::clamp(audible_beads::ControlLayer::finite(inputs[ASSIGN_CV_INPUT].getVoltage()) / 5.f, -1.f, 1.f) : 0.f, .1f);
            assignCv = smooth[17].value;
            for (int i = 0; i < 3; ++i) {
                smooth[9 + i].process(params[mixIds[i]].getValue(), .05f);
                mixPots[i].ProcessControlRate(smooth[9 + i].value);
            }
        }
        uiPhase += 1000.0 / std::max(double(args.sampleRate), 1.0);
        if (uiPhase >= 1.0) {
            uiPhase -= 1.0;
            for (auto& pot : mixPots) pot.ProcessUIRate();
        }
        if (down[0] && mixPots[0].editing_hidden_parameter()) {
            gainEdited = true;
            manualGain = std::clamp(1.f / 255.f + mixPots[0].hidden_parameter() * (254.f / 255.f), 1.f / 255.f, 1.f);
            gainManual = true;
            menuManualGain.store(manualGain, std::memory_order_relaxed);
            menuGainManual.store(1, std::memory_order_relaxed);
        }
        if (down[3]) {
            for (int i = 0; i < 3; ++i) if (mixPots[i].editing_hidden_parameter()) {
                assignmentEdited[i] = true;
                assignmentAmounts[i] = mixPots[i].hidden_parameter();
                menuAssignmentAmounts[i].store(assignmentAmounts[i], std::memory_order_relaxed);
                assignment = 3; menuAssignment.store(3, std::memory_order_relaxed);
            }
        }
        audible_beads::Controls controls;
        controls.media = media;
        controls.gain = gainManual ? uint8_t(std::clamp(int(std::lround(manualGain * 255.f)), 1, 255)) : 0;
        controls.freezeLatch = freezeLatch;
        controls.seed = seedLocked ? audible_beads::ControlLayer::SEED_LOCKED :
            (down[2] ? audible_beads::ControlLayer::SEED_ON : audible_beads::ControlLayer::SEED_OFF);
        controls.restartGain = restartGainSamples > 0;
        auto grain = [&](int index, bool pitch) {
            float pot = grainBase[index];
            float cv = grainCv[index];
            float amount = grainAmount[index];
            if (pitch) {
                controls.grain.pitch.base = pot;
                controls.grain.pitch.cv = cv;
                controls.grain.pitch.attenuverter = amount;
                controls.grain.pitch.has_cv = inputs[PITCH_CV_INPUT].isConnected();
            } else {
                beads::RandomizedParameter* p = index == 0 ? &controls.grain.time :
                    index == 1 ? &controls.grain.size : &controls.grain.shape;
                const int cvIds[3] = {TIME_CV_INPUT, SIZE_CV_INPUT, SHAPE_CV_INPUT};
                p->base = pot; p->cv = cv; p->attenuverter = amount;
                p->has_cv = inputs[cvIds[index]].isConnected();
            }
        };
        grain(0, false); grain(1, false); grain(2, false); grain(3, true);
        controls.density.base = audible_beads::ControlLayer::clamp01(densityBase);
        controls.density.cv = densityCv;
        controls.mix.feedback = mixValue(0, FEEDBACK_PARAM);
        controls.mix.dry_wet = mixValue(1, DRY_WET_PARAM);
        controls.mix.reverb_amount = mixValue(2, REVERB_PARAM);
        controls.mix.output_gate = gateOutput;
        audible_beads::Input input;
        input.leftConnected = inputs[AUDIO_L_INPUT].isConnected();
        input.rightConnected = inputs[AUDIO_R_INPUT].isConnected();
        input.left = input.leftConnected ? inputs[AUDIO_L_INPUT].getVoltage() : 0.f;
        input.right = input.rightConnected ? inputs[AUDIO_R_INPUT].getVoltage() : 0.f;
        input.seedConnected = inputs[SEED_INPUT].isConnected();
        input.seed = input.seedConnected ? inputs[SEED_INPUT].getVoltage() : 0.f;
        input.freeze = inputs[FREEZE_INPUT].isConnected() ? inputs[FREEZE_INPUT].getVoltage() : 0.f;
        seedGateLight = updateGateLight(seedGateLight, input.seedConnected ? input.seed : 0.f);
        freezeGateLight = updateGateLight(freezeGateLight, input.freeze);
        auto out = supportedRate ? engine->process(input, controls,
            outputs[AUDIO_R_OUTPUT].isConnected() || gateOutput) : audible_beads::Output{};
        statusSynth.store(engine->synthMode(), std::memory_order_relaxed);
        if (restartGainSamples && supportedRate) --restartGainSamples;
        outputs[AUDIO_L_OUTPUT].setVoltage(out.left);
        outputs[AUDIO_R_OUTPUT].setVoltage(out.right);
        outputs[AUDIO_L_OUTPUT].setChannels(1);
        outputs[AUDIO_R_OUTPUT].setChannels(1);
        lights[FREEZE_LIGHT].setBrightness(freezeLatch || down[1] || freezeGateLight ? 1.f : 0.f);
        // A held/gated button is solid; a latched button slowly breathes.
        // Keep the trough visible on the light-colored hardware-style cap.
        seedIndicatorPhase += double(args.sampleTime) / 2.048;
        if (seedIndicatorPhase >= 1.) seedIndicatorPhase -= 1.;
        const float triangle = float(1. - std::fabs(2. * seedIndicatorPhase - 1.));
        lights[SEED_LIGHT].setBrightness(down[2] || seedGateLight ? 1.f :
            (seedLocked ? .55f + .45f * triangle : 0.f));
        lights[ASSIGN_1_LIGHT].setBrightness(assignment == 0 ? .8f : assignment == 3 ? assignmentAmounts[0] : 0.f);
        lights[ASSIGN_2_LIGHT].setBrightness(assignment == 1 ? .8f : assignment == 3 ? assignmentAmounts[1] : 0.f);
        lights[ASSIGN_3_LIGHT].setBrightness(assignment == 2 ? .8f : assignment == 3 ? assignmentAmounts[2] : 0.f);
        const int activeMedia = int(engine->media());
        const float mediaBrightness = activeMedia == media || ((sampleIndex / 12000) & 1) ? .85f : .25f;
        const float red[4] = {1.f, 1.f, 0.f, .75f};
        const float green[4] = {0.f, .42f, 1.f, .75f};
        const float blue[4] = {.18f, 0.f, 1.f, .85f};
        lights[MEDIA_R_LIGHT].setBrightness(red[media] * mediaBrightness);
        lights[MEDIA_G_LIGHT].setBrightness(green[media] * mediaBrightness);
        lights[MEDIA_B_LIGHT].setBrightness(blue[media] * mediaBrightness);
        lights[VU_LIGHT].setBrightness(float(engine->vuMeter()) / 255.f);
        lights[ERROR_LIGHT].setBrightness(engine->hostRate() == int(std::lround(args.sampleRate)) ? 0.f : 1.f);
        ++sampleIndex;
    }

    void processBypass(const ProcessArgs&) override {
        audible_beads::Input input;
        input.leftConnected = inputs[AUDIO_L_INPUT].isConnected();
        input.rightConnected = inputs[AUDIO_R_INPUT].isConnected();
        input.left = input.leftConnected ? inputs[AUDIO_L_INPUT].getVoltage() : 0.f;
        input.right = input.rightConnected ? inputs[AUDIO_R_INPUT].getVoltage() : 0.f;
        const auto output = audible_beads::BeadsEngine::bypass(input, outputs[AUDIO_R_OUTPUT].isConnected());
        outputs[AUDIO_L_OUTPUT].setVoltage(output.left);
        outputs[AUDIO_R_OUTPUT].setVoltage(output.right);
        outputs[AUDIO_L_OUTPUT].setChannels(1);
        outputs[AUDIO_R_OUTPUT].setChannels(1);
    }

    float mixValue(int index, int paramId) {
        (void)paramId;
        float pot = audible_beads::ControlLayer::clamp01(mixPots[index].main_parameter());
        float amount = assignment == 3 ? assignmentAmounts[index] : (assignment == index ? 1.f : 0.f);
        float cv = assignCv;
        return std::clamp(pot + cv * amount, 0.f, 1.f);
    }

    static bool updateGateLight(bool high, float volts) {
        if (!std::isfinite(volts)) volts = 0.f;
        return high ? volts > .2919571f : volts >= .3804290f;
    }

    void updateButtons(const ProcessArgs& args) {
        const int ids[4] = {MEDIA_PARAM, FREEZE_PARAM, SEED_PARAM, ASSIGN_PARAM};
        bool pressedNow[4]{};
        for (int i = 0; i < 4; ++i) pressedNow[i] = params[ids[i]].getValue() > .5f;
        for (int i = 0; i < 4; ++i) {
            if (pressedNow[i] && !previousButton[i]) {
                down[i] = true; held[i] = 0;
                if (!comboConsumed && ((i == 1 && pressedNow[2]) || (i == 2 && pressedNow[1]))) {
                    seedLocked = true; menuSeedLocked.store(1); comboConsumed = true;
                } else if (!comboConsumed && ((i == 1 && pressedNow[3]) || (i == 3 && pressedNow[1]))) {
                    // Firmware uses ASSIGN+FREEZE to toggle audio autosave;
                    // that state belongs to the later persistence part.
                    comboConsumed = true;
                } else if (!comboConsumed && ((i == 2 && pressedNow[3]) || (i == 3 && pressedNow[2]))) {
                    gateOutput = !gateOutput; menuGateOutput.store(gateOutput); comboConsumed = true;
                }
                if (i == 2 && !comboConsumed && seedLocked) {
                    seedLocked = false; menuSeedLocked.store(0);
                }
                if (i == 0) { mixPots[0].Lock(); gainEdited = false; }
                if (i == 3) {
                    for (int k = 0; k < 3; ++k) { mixPots[k].Lock(); assignmentEdited[k] = false; }
                }
            }
            if (down[i] && pressedNow[i]) {
                ++held[i];
            }
            if (!pressedNow[i] && previousButton[i]) {
                down[i] = false;
                if (!comboConsumed) {
                    if (i == 0 && !gainEdited) {
                        if (held[i] >= uint64_t(args.sampleRate * 1.5)) {
                            gainManual = false;
                            menuGainManual.store(0, std::memory_order_relaxed);
                            restartGainSamples = uint64_t(std::ceil(args.sampleRate * .020));
                            mediaLongHandled = true;
                        } else {
                            media = (media + 3) & 3;
                            menuMedia.store(media);
                        }
                    }
                    else if (i == 1) { freezeLatch = !freezeLatch; menuFreezeLatch.store(freezeLatch); }
                    else if (i == 2) {
                        // Firmware recognizes a four-second hold on release.
                        seedLocked = held[i] >= uint64_t(args.sampleRate * 4.0);
                        menuSeedLocked.store(seedLocked);
                    } else if (i == 3) {
                        bool edited = assignmentEdited[0] || assignmentEdited[1] || assignmentEdited[2];
                        if (!edited) { assignment = assignment == 3 ? 0 : (assignment + 1) % 3; menuAssignment.store(assignment); }
                    }
                }
                if (i == 0) mixPots[0].Unlock();
                if (i == 3) for (auto& pot : mixPots) pot.Unlock();
                held[i] = 0;
                if (i == 0) mediaLongHandled = false;
            }
            previousButton[i] = pressedNow[i];
        }
        // ASSIGN+FREEZE is the firmware autosave gesture. Autosave is outside
        // this module port, so consume the complete chord without triggering
        // either button's individual action.
        if (comboConsumed && !pressedNow[1] && !pressedNow[2] && !pressedNow[3]) comboConsumed = false;
    }

    void addMenuItems(Menu* menu) {
        menu->addChild(createMenuLabel(statusSynth.load() ? "Wavetable synthesis (FREEZE stops new grains)" : "Recorded audio (FREEZE holds the buffer)"));
        menu->addChild(createSubmenuItem("Edit knobs", "", [=](Menu* m) {
            const char* labels[] = {"Normal mix controls", "Input gain (red knob)", "CV assignment amounts (green knobs)"};
            for (int view = 0; view < 3; ++view)
                m->addChild(createCheckMenuItem(labels[view], "", [=] { return editView.load() == view; }, [=] { editView.store(view); }));
        }));
        menu->addChild(createSubmenuItem("Media / quality", "", [=](Menu* m) {
            const char* labels[] = {"Scorched cassette", "Sunny tape", "Cold digital", "Bright digital"};
            for (int q = 0; q < 4; ++q) m->addChild(createCheckMenuItem(
                labels[q], "", [=] { return menuMedia.load() == q; }, [=] { menuMedia.store(q); }));
        }));
        menu->addChild(createCheckMenuItem("Seed button latched", "", [=] { return menuSeedLocked.load(); },
            [=] { menuSeedLocked.store(!menuSeedLocked.load()); }));
        menu->addChild(createCheckMenuItem("Manual input gain", "", [=] { return menuGainManual.load(); },
            [=] { menuGainManual.store(!menuGainManual.load()); }));
        menu->addChild(createSubmenuItem("Manual input gain value", "", [=](Menu* m) {
            for (int i = 0; i < 5; ++i) {
                const float value = kMenuGainValues[i];
                m->addChild(createCheckMenuItem(kMenuGainLabels[i], "",
                [=] { return std::fabs(menuManualGain.load() - value) < .005f; }, [=] {
                    getParamQuantity(INPUT_GAIN_PARAM)->setImmediateValue(value);
                    menuManualGain.store(value); menuGainManual.store(1);
                }));
            }
        }));
        menu->addChild(createSubmenuItem("CV assignment", "", [=](Menu* m) {
            const char* labels[] = {"Feedback", "Dry/wet", "Reverb", "Custom amounts"};
            for (int a = 0; a < 3; ++a) m->addChild(createCheckMenuItem(labels[a], "",
                [=] { return menuAssignment.load() == a; }, [=] { menuAssignment.store(a); }));
            m->addChild(createSubmenuItem(labels[3], "", [=](Menu* custom) {
                const char* destinations[] = {"Feedback amount", "Dry/wet amount", "Reverb amount"};
                for (int k = 0; k < 3; ++k) custom->addChild(createSubmenuItem(destinations[k], "", [=](Menu* amounts) {
                    for (int j = 0; j < 5; ++j) {
                        const float value = kMenuAmountValues[j];
                        amounts->addChild(createCheckMenuItem(kMenuAmountLabels[j], "",
                        [=] { return std::fabs(menuAssignmentAmounts[k].load() - value) < .005f; }, [=] {
                            getParamQuantity(FEEDBACK_CV_AMOUNT_PARAM + k)->setImmediateValue(value);
                            menuAssignmentAmounts[k].store(value); menuAssignment.store(3);
                        }));
                    }
                }));
            }));
        }));
        menu->addChild(createCheckMenuItem("R output: grain gate", "", [=] { return menuGateOutput.load(); },
            [=] { menuGateOutput.store(!menuGateOutput.load()); }));
    }
};

// Match Marbles' CKD6Light<GreenLight>, including its opaque light face
// and Rack's normal light/halo blending.
struct BeadsButtonLight : GreenLight {
    BeadsButtonLight() { box.size = Vec(22.f, 22.f); }
};
template <typename Base>
struct BeadsKnob : Base {
    BeadsKnob() {
        // The printed envelope symbols sit close to the hardware knobs.
        // Suppress Rack's extra drop shadow so it does not obscure them.
        this->shadow->box.size = Vec();
    }
};

template <bool Gain>
struct BeadsEditKnob : Trimpot {
    void draw(const DrawArgs& args) override {
        nvgBeginPath(args.vg);
        nvgCircle(args.vg, box.size.x / 2.f, box.size.y / 2.f, box.size.x / 2.f + 1.f);
        nvgStrokeWidth(args.vg, 2.f);
        nvgStrokeColor(args.vg, Gain ? nvgRGB(224, 45, 75) : nvgRGB(87, 172, 61));
        nvgStroke(args.vg);
        Trimpot::draw(args);
    }
};

struct BeadsWidget : ModuleWidget {
    ParamWidget* mixKnobs[3]{};
    ParamWidget* gainKnob = nullptr;
    ParamWidget* amountKnobs[3]{};

    explicit BeadsWidget(Beads* module) {
        namespace p = audible_beads::panel;
        setModule(module);
        setPanel(Svg::load(asset::plugin(pluginInstance, "res/Beads.svg")));
        addChild(createWidgetCentered<ScrewSilver>(p::screwTopLeft()));
        addChild(createWidgetCentered<ScrewSilver>(p::screwTopRight()));
        addChild(createWidgetCentered<ScrewSilver>(p::screwBottomLeft()));
        addChild(createWidgetCentered<ScrewSilver>(p::screwBottomRight()));
        addParam(createParamCentered<TL1105>(p::mediaButton(), module, Beads::MEDIA_PARAM));
        addParam(createParamCentered<CKD6>(p::freezeButton(), module, Beads::FREEZE_PARAM));
        addParam(createParamCentered<CKD6>(p::seedButton(), module, Beads::SEED_PARAM));
        addParam(createParamCentered<TL1105>(p::assignButton(), module, Beads::ASSIGN_PARAM));
        addChild(createLightCentered<MediumLight<RedGreenBlueLight>>(p::mediaLight(), module, Beads::MEDIA_R_LIGHT));
        addChild(createLightCentered<BeadsButtonLight>(p::freezeButton(), module, Beads::FREEZE_LIGHT));
        addChild(createLightCentered<BeadsButtonLight>(p::seedButton(), module, Beads::SEED_LIGHT));
        addChild(createLightCentered<MediumLight<YellowLight>>(p::feedbackLight(), module, Beads::ASSIGN_1_LIGHT));
        addChild(createLightCentered<MediumLight<YellowLight>>(p::dryWetLight(), module, Beads::ASSIGN_2_LIGHT));
        addChild(createLightCentered<MediumLight<YellowLight>>(p::reverbLight(), module, Beads::ASSIGN_3_LIGHT));
        addParam(createParamCentered<BeadsKnob<Rogan3PSWhite>>(p::time(), module, Beads::TIME_PARAM));
        addParam(createParamCentered<BeadsKnob<Rogan3PSWhite>>(p::pitch(), module, Beads::PITCH_PARAM));
        addParam(createParamCentered<BeadsKnob<Rogan2PSWhite>>(p::size(), module, Beads::SIZE_PARAM));
        addParam(createParamCentered<BeadsKnob<Rogan2PSWhite>>(p::shape(), module, Beads::SHAPE_PARAM));
        addParam(createParamCentered<BeadsKnob<Rogan3PSWhite>>(p::density(), module, Beads::DENSITY_PARAM));
        addParam(createParamCentered<Trimpot>(p::timeAmount(), module, Beads::TIME_AMOUNT_PARAM));
        addParam(createParamCentered<Trimpot>(p::sizeAmount(), module, Beads::SIZE_AMOUNT_PARAM));
        addParam(createParamCentered<Trimpot>(p::shapeAmount(), module, Beads::SHAPE_AMOUNT_PARAM));
        addParam(createParamCentered<Trimpot>(p::pitchAmount(), module, Beads::PITCH_AMOUNT_PARAM));
        mixKnobs[0] = createParamCentered<Trimpot>(p::feedback(), module, Beads::FEEDBACK_PARAM);
        mixKnobs[1] = createParamCentered<Trimpot>(p::dryWet(), module, Beads::DRY_WET_PARAM);
        mixKnobs[2] = createParamCentered<Trimpot>(p::reverb(), module, Beads::REVERB_PARAM);
        for (auto* knob : mixKnobs) addParam(knob);
        gainKnob = createParamCentered<BeadsEditKnob<true>>(p::feedback(), module, Beads::INPUT_GAIN_PARAM);
        gainKnob->visible = false; addParam(gainKnob);
        amountKnobs[0] = createParamCentered<BeadsEditKnob<false>>(p::feedback(), module, Beads::FEEDBACK_CV_AMOUNT_PARAM);
        amountKnobs[1] = createParamCentered<BeadsEditKnob<false>>(p::dryWet(), module, Beads::DRY_WET_CV_AMOUNT_PARAM);
        amountKnobs[2] = createParamCentered<BeadsEditKnob<false>>(p::reverb(), module, Beads::REVERB_CV_AMOUNT_PARAM);
        for (auto* knob : amountKnobs) { knob->visible = false; addParam(knob); }
        // Share the hardware VU aperture with the unsupported-rate diagnostic.
        addChild(createLightCentered<MediumLight<GreenRedLight>>(p::vuLight(), module, Beads::VU_LIGHT));
        addInput(createInputCentered<PJ301MPort>(p::timeCv(), module, Beads::TIME_CV_INPUT));
        addInput(createInputCentered<PJ301MPort>(p::sizeCv(), module, Beads::SIZE_CV_INPUT));
        addInput(createInputCentered<PJ301MPort>(p::shapeCv(), module, Beads::SHAPE_CV_INPUT));
        addInput(createInputCentered<PJ301MPort>(p::pitchCv(), module, Beads::PITCH_CV_INPUT));
        addInput(createInputCentered<PJ301MPort>(p::densityCv(), module, Beads::DENSITY_CV_INPUT));
        addInput(createInputCentered<PJ301MPort>(p::assignCv(), module, Beads::ASSIGN_CV_INPUT));
        addInput(createInputCentered<PJ301MPort>(p::freezeGate(), module, Beads::FREEZE_INPUT));
        addInput(createInputCentered<PJ301MPort>(p::seedGate(), module, Beads::SEED_INPUT));
        addInput(createInputCentered<PJ301MPort>(p::audioL(), module, Beads::AUDIO_L_INPUT));
        addInput(createInputCentered<PJ301MPort>(p::audioR(), module, Beads::AUDIO_R_INPUT));
        addOutput(createOutputCentered<PJ301MPort>(p::outL(), module, Beads::AUDIO_L_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(p::outR(), module, Beads::AUDIO_R_OUTPUT));
    }
    void step() override {
        auto* beads = dynamic_cast<Beads*>(module);
        const int view = beads ? beads->editView.load() : 0;
        gainKnob->visible = view == 1;
        for (int i = 0; i < 3; ++i) {
            mixKnobs[i]->visible = view != 2 && (i != 0 || view != 1);
            amountKnobs[i]->visible = view == 2;
        }
        ModuleWidget::step();
    }
    void appendContextMenu(Menu* menu) override {
        auto* beads = dynamic_cast<Beads*>(module);
        if (beads) beads->addMenuItems(menu);
    }
};

Model* modelBeads = createModel<Beads, BeadsWidget>("Beads");
