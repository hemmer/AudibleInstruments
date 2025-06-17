#include "plugin.hpp"
#include "Ripples/ripples.hpp"

typedef ripples::RipplesEngine<false> RipplesEngineV2;


struct RipplesV2 : Module {
	enum ParamId {
		SLOPE_PARAM,
		FREQUENCY_PARAM,
		RESONANCE_PARAM,
		GAIN_PARAM,
		FM_PARAM,
		PARAMS_LEN
	};
	enum InputId {
		IN1_INPUT,
		FM_INPUT,
		RESO_INPUT,
		IN2_INPUT,
		VOCT_INPUT,
		GAIN_INPUT,
		INPUTS_LEN
	};
	enum OutputId {
		HP_OUTPUT,
		BP_OUTPUT,
		LP_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		IN1_LIGHT,
		IN2_LIGHT,
		LIGHTS_LEN
	};

	RipplesEngineV2 engines[16];

	dsp::VuMeter2 vuMeter[2];
	dsp::ClockDivider vuDivider;
	const static int VU_UPDATE_RATE = 32;

	RipplesV2() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(RESONANCE_PARAM, 0.f, 1.f, 0.f, "Resonance", "%", 0, 100);
		configParam(FREQUENCY_PARAM, std::log2(ripples::kFreqKnobMin), std::log2(ripples::kFreqKnobMax), std::log2(ripples::kFreqKnobMax), "Frequency", " Hz", 2.f);
		configParam(FM_PARAM, -1.f, 1.f, 0.f, "Frequency modulation", "%", 0, 100);

		configSwitch(SLOPE_PARAM, 0.f, 1.f, 0.f, "Slope", {"2-pole", "4-pole"});
		// note this is 40 dB, not 20 dB as we square the gain value below to get the right taper
		configParam(GAIN_PARAM, 0.0, 2.5, 1.0, "Gain", " dB", -10, 40);

		configInput(FM_INPUT, "FM");
		configInput(IN2_INPUT, "In 2");
		configInput(RESO_INPUT, "Resonance CV");
		configInput(VOCT_INPUT, "V/Oct");
		configInput(IN1_INPUT, "In 1");
		configInput(GAIN_INPUT, "Gain");
		configOutput(LP_OUTPUT, "Lowpass");
		configOutput(BP_OUTPUT, "Bandpass");
		configOutput(HP_OUTPUT, "Highpass");

		vuDivider.setDivision(VU_UPDATE_RATE);

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

	// clipping: https://falstad.com/circuit/circuitjs.html?ctz=CQAgjCAMB0l3BWEAWATLBBOAzM5kxVJsAObbEBJJZChAUwFowwAoAJREaJPE1S6oEANj4DI4FBIDs0JBIVzWAJ0GReYfl0IatEsPDgrtqXQOamQQ0foRGA5id7WnV6eKisA7q83nkZp5g0shc2AKo7lwB4JYQBvBQ0JiYdobpcBDs9ADOAJY5AC4AhgB2AMb0rMUg4bEaePUgGkjMSAmGSYYIIZABmuFwwsJkUODp3mECLAJ1M56Oc41zep4+jHOEU80Kk2CNJPqNOmvbh9t1kMYbswIxl+OGxvfTyx4IHXtv4Ae7qvuhE6ofBNfTwVgAeVqCFeoWQYFEALGV1UNzc5jqgzGVHBjjRWPxMORrAAXmcJDFzgIACb0ABmxQArgAbQqMEn0Ur0ZSkn6hc6UiQ0+lM1nsznc6pWEFI7BEpEtbTtDJcGDdXrYYTSQjIYbwzBjDpGHzA-SWU0gTA2SZygRW6GwhbS-TCIUy1Yo52W0S272G8EAe3AomEoQpkEwzkUEAk8mRtRA0jGSewrCAA

	void process(const ProcessArgs& args) override {

		const int channels = std::max({inputs[IN1_INPUT].getChannels(), inputs[IN2_INPUT].getChannels(),
		                               inputs[VOCT_INPUT].getChannels(), 1});

		const float gain = params[GAIN_PARAM].getValue() * params[GAIN_PARAM].getValue();
		float channel1Sum = 0.f;
		float channel2Sum = 0.f;

		// Reuse the same frame object for multiple engines because the params aren't touched.
		RipplesEngineV2::Frame frame;
		frame.res_knob = params[RESONANCE_PARAM].getValue();
		frame.freq_knob = rescale(params[FREQUENCY_PARAM].getValue(), std::log2(ripples::kFreqKnobMin), std::log2(ripples::kFreqKnobMax), 0.f, 1.f);
		frame.fm_knob = params[FM_PARAM].getValue();
		frame.gain_cv_present = inputs[GAIN_INPUT].isConnected();
		frame.slope = (int)params[SLOPE_PARAM].getValue();

		for (int c = 0; c < channels; c++) {
			frame.res_cv = inputs[RESO_INPUT].getPolyVoltage(c);
			frame.freq_cv = inputs[VOCT_INPUT].getPolyVoltage(c);
			frame.fm_cv = inputs[FM_INPUT].getPolyVoltage(c);

			frame.input = gain * inputs[IN1_INPUT].getVoltage(c);
			frame.input2 = inputs[IN2_INPUT].getVoltage(c);
			frame.gain_cv = inputs[GAIN_INPUT].getPolyVoltage(c);

			// for vuMeter
			channel1Sum += clamp(std::abs(gain * inputs[IN1_INPUT].getVoltage(c)), 0.f, ripples::clipFactor) / 5.f;
			channel2Sum += std::abs(inputs[IN2_INPUT].getVoltage(c) / 5.f);


			engines[c].process(frame);

			outputs[HP_OUTPUT].setVoltage(frame.hp, c);
			outputs[BP_OUTPUT].setVoltage(frame.bp, c);
			outputs[LP_OUTPUT].setVoltage(frame.gain_cv_present ? frame.lpvca : -frame.lp, c);
		}

		if (vuDivider.process()) {
			const float sampleTime = args.sampleTime * VU_UPDATE_RATE;

			vuMeter[0].process(sampleTime, channel1Sum / channels);
			vuMeter[1].process(sampleTime, channel2Sum / channels);

			lights[IN1_LIGHT].setBrightnessSmooth(vuMeter[0].getBrightness(-6.f, 0.f), sampleTime);
			lights[IN2_LIGHT].setBrightnessSmooth(vuMeter[1].getBrightness(-6.f, 0.f), sampleTime);
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

		addParam(createParam<CKSS>(mm2px(Vec(30.107, 21.706)), module, RipplesV2::SLOPE_PARAM));
		addParam(createParamCentered<Rogan3PSWhite>(mm2px(Vec(13.302, 25.349)), module, RipplesV2::FREQUENCY_PARAM));
		addParam(createParamCentered<Rogan2PSWhite>(mm2px(Vec(30.453, 46.95)), module, RipplesV2::RESONANCE_PARAM));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(8.052, 66.0)), module, RipplesV2::GAIN_PARAM));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(20.152, 66.0)), module, RipplesV2::FM_PARAM));

		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.052, 81.9)), module, RipplesV2::IN1_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(20.152, 81.9)), module, RipplesV2::FM_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(32.252, 81.9)), module, RipplesV2::RESO_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.052, 96.5)), module, RipplesV2::IN2_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(20.152, 96.5)), module, RipplesV2::VOCT_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(32.252, 96.5)), module, RipplesV2::GAIN_INPUT));

		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(8.052, 111.1)), module, RipplesV2::HP_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(20.152, 111.1)), module, RipplesV2::BP_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(32.252, 111.1)), module, RipplesV2::LP_OUTPUT));

		addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(4.702, 75.55)), module, RipplesV2::IN1_LIGHT));
		addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(4.702, 90.15)), module, RipplesV2::IN2_LIGHT));
	}
};


Model* modelRipplesV2 = createModel<RipplesV2, RipplesV2Widget>("RipplesV2");