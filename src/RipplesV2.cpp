#include "plugin.hpp"
#include "Ripples/ripples.hpp"

typedef ripples::RipplesEngine<false> RipplesEngineV2;


struct RipplesV2 : Module {
	enum ParamId {
		SLOPE_PARAM,
		FREQ_PARAM,
		RES_PARAM,
		GAIN_PARAM,
		FM_PARAM,
		PARAMS_LEN
	};
	enum InputId {
		FM_INPUT,
		IN2_INPUT,
		RES_INPUT,
		FREQ_INPUT,
		IN1_INPUT,
		GAIN_INPUT,
		INPUTS_LEN
	};
	enum OutputId {
		LP_OUTPUT,
		BP_OUTPUT,
		HP_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		IN1_LIGHT,
		IN2_LIGHT,
		LIGHTS_LEN
	};

	RipplesEngineV2 engines[16];

	RipplesV2() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(RES_PARAM, 0.f, 1.f, 0.f, "Resonance", "%", 0, 100);
		configParam(FREQ_PARAM, std::log2(ripples::kFreqKnobMin), std::log2(ripples::kFreqKnobMax), std::log2(ripples::kFreqKnobMax), "Frequency", " Hz", 2.f);
		configParam(FM_PARAM, -1.f, 1.f, 0.f, "Frequency modulation", "%", 0, 100);

		configSwitch(SLOPE_PARAM, 0.f, 1.f, 0.f, "Slope", {"2-pole", "4-pole"});

		configParam(GAIN_PARAM, 0.f, 1.f, 0.f, "Gain");
		configInput(FM_INPUT, "FM");
		configInput(IN2_INPUT, "In 2");
		configInput(RES_INPUT, "Resonance CV");
		configInput(FREQ_INPUT, "V/Oct");
		configInput(IN1_INPUT, "In 1");
		configInput(GAIN_INPUT, "Gain");
		configOutput(LP_OUTPUT, "Lowpass");
		configOutput(BP_OUTPUT, "Bandpass");
		configOutput(HP_OUTPUT, "Highpass");

		onSampleRateChange();
	}

	void onReset() override {
		onSampleRateChange();
	}

	void onSampleRateChange() override {
		// TODO In Rack v2, replace with args.sampleRate
		for (int c = 0; c < 16; c++) {
			engines[c].setSampleRate(APP->engine->getSampleRate());
		}
	}

	void process(const ProcessArgs& args) override {

		const int channels = std::max({inputs[IN1_INPUT].getChannels(), inputs[IN2_INPUT].getChannels(), 
									   inputs[FREQ_INPUT].getChannels(), 1});

		// Reuse the same frame object for multiple engines because the params aren't touched.
		RipplesEngineV2::Frame frame;
		frame.res_knob = params[RES_PARAM].getValue();
		frame.freq_knob = rescale(params[FREQ_PARAM].getValue(), std::log2(ripples::kFreqKnobMin), std::log2(ripples::kFreqKnobMax), 0.f, 1.f);
		frame.fm_knob = params[FM_PARAM].getValue();
		frame.gain_cv_present = inputs[GAIN_INPUT].isConnected();
		frame.slope = (int)params[SLOPE_PARAM].getValue();

		for (int c = 0; c < channels; c++) {
			frame.res_cv = inputs[RES_INPUT].getPolyVoltage(c);
			frame.freq_cv = inputs[FREQ_INPUT].getPolyVoltage(c);
			frame.fm_cv = inputs[FM_INPUT].getPolyVoltage(c);
			frame.input = params[GAIN_PARAM].getValue() * inputs[IN1_INPUT].getVoltage(c) + inputs[IN2_INPUT].getVoltage(c);
			frame.gain_cv = inputs[GAIN_INPUT].getPolyVoltage(c);

			engines[c].process(frame);

			outputs[HP_OUTPUT].setVoltage(frame.hp, c);
			outputs[BP_OUTPUT].setVoltage(frame.bp, c);
			outputs[LP_OUTPUT].setVoltage(frame.gain_cv_present ? frame.lpvca : frame.lp, c);
		}

		outputs[HP_OUTPUT].setChannels(channels);
		outputs[BP_OUTPUT].setChannels(channels);
		outputs[LP_OUTPUT].setChannels(channels);
	}
};


struct RipplesV2Widget : ModuleWidget {
	RipplesV2Widget(RipplesV2* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/RipplesV2.svg")));

		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createParam<CKSS>(mm2px(Vec(30.334, 21.656)), module, RipplesV2::SLOPE_PARAM));
		addParam(createParamCentered<Rogan3PSWhite>(mm2px(Vec(13.109, 24.381)), module, RipplesV2::FREQ_PARAM));
		addParam(createParamCentered<Rogan2PSWhite>(mm2px(Vec(30.466, 46.532)), module, RipplesV2::RES_PARAM));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(8.12, 65.677)), module, RipplesV2::GAIN_PARAM));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(20.26, 65.677)), module, RipplesV2::FM_PARAM));

		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(20.655, 82.05)), module, RipplesV2::FM_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.682, 82.274)), module, RipplesV2::IN1_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(32.628, 82.293)), module, RipplesV2::RES_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(20.655, 96.334)), module, RipplesV2::FREQ_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.682, 96.705)), module, RipplesV2::IN2_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(32.628, 96.563)), module, RipplesV2::GAIN_INPUT));

		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(8.682, 111.05)), module, RipplesV2::HP_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(20.655, 111.05)), module, RipplesV2::BP_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(32.628, 111.05)), module, RipplesV2::LP_OUTPUT));

		addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(4.801, 75.315)), module, RipplesV2::IN1_LIGHT));
		addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(4.786, 89.909)), module, RipplesV2::IN2_LIGHT));
	}
};


Model* modelRipplesV2 = createModel<RipplesV2, RipplesV2Widget>("RipplesV2");