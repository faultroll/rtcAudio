
# utility
srcs += \
        modules/include/channel_layout.cc \
        modules/include/audio_frame.cc \
        modules/include/audio_frame_operations.cc
srcs += \
        modules/audio_processing/utility/block_mean_calculator.cc \
        modules/audio_processing/utility/pffft_wrapper.cc \
        modules/audio_processing/splitting_filter.cc \
        modules/audio_processing/three_band_filter_bank.cc \
        modules/audio_processing/level_estimator_impl.cc \
        modules/audio_processing/rms_level.cc \
        modules/audio_processing/low_cut_filter.cc \
        modules/audio_processing/utility/cascaded_biquad_filter.cc \
        modules/audio_processing/high_pass_filter.cc \
        modules/audio_processing/logging/apm_data_dumper.cc \
        modules/audio_processing/audio_buffer.cc
srcs_c += \
        modules/audio_processing/utility/delay_estimator.c \
        modules/audio_processing/utility/delay_estimator_wrapper.c
# aecm
srcs_c += \
        modules/audio_processing/aecm/aecm_core.c \
        modules/audio_processing/aecm/aecm_core_c.c \
        modules/audio_processing/aecm/aecm_core_mips.c \
        modules/audio_processing/aecm/aecm_core_neon.c \
        modules/audio_processing/aecm/echo_control_mobile.c
srcs += \
        modules/audio_processing/aecm/echo_control_mobile_impl.cc
# aec
srcs += \
        modules/audio_processing/aec/aec_core_c.cc \
        modules/audio_processing/aec/aec_core_mips.cc \
        modules/audio_processing/aec/aec_core_neon.cc \
        modules/audio_processing/aec/aec_core_sse2.cc \
        modules/audio_processing/aec/aec_resampler.c \
        modules/audio_processing/aec/aec_core.cc \
        modules/audio_processing/aec/echo_cancellation.cc
srcs += \
        modules/audio_processing/aec/echo_cancellation_impl.cc
# agc
srcs_c += \
        modules/audio_processing/agc/legacy/analog_agc.c \
        modules/audio_processing/agc/legacy/digital_agc.c
srcs += \
        modules/audio_processing/agc/agc.cc \
        modules/audio_processing/agc/loudness_histogram.cc \
        modules/audio_processing/agc/utility.cc \
        modules/audio_processing/agc/agc_manager_direct.cc \
        modules/audio_processing/agc/gain_control_impl.cc \
        # modules/audio_processing/agc/gain_control_for_experimental_agc.cc
# ns
srcs_c += \
        modules/audio_processing/ns/noise_suppression.c \
        modules/audio_processing/ns/noise_suppression_x.c \
        modules/audio_processing/ns/ns_core.c \
        modules/audio_processing/ns/nsx_core.c \
        modules/audio_processing/ns/nsx_core_c.c \
        modules/audio_processing/ns/nsx_core_mips.c \
        modules/audio_processing/ns/nsx_core_neon.c
srcs += \
        modules/audio_processing/ns/noise_suppression_impl.cc
# echo_detector
srcs += \
        modules/audio_processing/echo_detector/circular_buffer.cc \
        modules/audio_processing/echo_detector/mean_variance_estimator.cc \
        modules/audio_processing/echo_detector/moving_max.cc \
        modules/audio_processing/echo_detector/normalized_covariance_estimator.cc \
        modules/audio_processing/echo_detector/residual_echo_detector.cc
# agc2(level_controller)
srcs_cpl_only += \
        modules/audio_processing/agc2/biquad_filter.cc \
        modules/audio_processing/agc2/down_sampler.cc \
        modules/audio_processing/agc2/noise_level_estimator.cc \
        modules/audio_processing/agc2/noise_spectrum_estimator.cc \
        modules/audio_processing/agc2/legacy/gain_selector.cc \
        modules/audio_processing/agc2/legacy/peak_level_estimator.cc \
        modules/audio_processing/agc2/legacy/saturating_gain_estimator.cc \
        modules/audio_processing/agc2/signal_classifier.cc \
        modules/audio_processing/agc2/legacy/gain_applier.cc \
        modules/audio_processing/agc2/legacy/gain_curve_applier.cc \
        modules/audio_processing/agc2/legacy/level_controller.cc
# agc2(gain_controller2)
srcs_cpl_only += \
        modules/audio_processing/agc2/gain_applier.cc \
        modules/audio_processing/agc2/adaptive_agc.cc \
        modules/audio_processing/agc2/adaptive_digital_gain_applier.cc \
        modules/audio_processing/agc2/adaptive_mode_level_estimator_agc.cc \
        modules/audio_processing/agc2/adaptive_mode_level_estimator.cc \
        modules/audio_processing/agc2/compute_interpolated_gain_curve.cc \
        modules/audio_processing/agc2/fixed_digital_level_estimator.cc \
        modules/audio_processing/agc2/interpolated_gain_curve.cc \
        modules/audio_processing/agc2/limiter_db_gain_curve.cc \
        modules/audio_processing/agc2/limiter.cc \
        modules/audio_processing/agc2/fixed_gain_controller.cc \
        modules/audio_processing/agc2/saturation_protector.cc \
        modules/audio_processing/agc2/vad_with_level.cc \
        modules/audio_processing/agc2/vector_float_frame.cc \
        modules/audio_processing/agc2/gain_controller2.cc
# agc2(rnn_vad)
srcs_cpl_only += \
        modules/audio_processing/agc2/rnn_vad/common.cc \
        modules/audio_processing/agc2/rnn_vad/features_extraction.cc \
        modules/audio_processing/agc2/rnn_vad/lp_residual.cc \
        modules/audio_processing/agc2/rnn_vad/pitch_search.cc \
        modules/audio_processing/agc2/rnn_vad/pitch_search_internal.cc \
        modules/audio_processing/agc2/rnn_vad/rnn.cc \
        modules/audio_processing/agc2/rnn_vad/spectral_features.cc \
        modules/audio_processing/agc2/rnn_vad/spectral_features_internal.cc \
        modules/audio_processing/agc2/rnn_vad/auto_correlation.cc \
        modules/audio_processing/agc2/rnn_vad/fft_util.cc \
        # modules/audio_processing/agc2/rnn_vad/rnn_vad_tool.cc
# ns2
srcs_cpl_only += \
        modules/audio_processing/ns2/fast_math.cc \
        modules/audio_processing/ns2/histograms.cc \
        modules/audio_processing/ns2/noise_estimator.cc \
        modules/audio_processing/ns2/noise_suppressor.cc \
        modules/audio_processing/ns2/ns_fft.cc \
        modules/audio_processing/ns2/prior_signal_model.cc \
        modules/audio_processing/ns2/prior_signal_model_estimator.cc \
        modules/audio_processing/ns2/quantile_noise_estimator.cc \
        modules/audio_processing/ns2/signal_model.cc \
        modules/audio_processing/ns2/signal_model_estimator.cc \
        modules/audio_processing/ns2/speech_probability_estimator.cc \
        modules/audio_processing/ns2/suppression_params.cc \
        modules/audio_processing/ns2/wiener_filter.cc
# aec3(aec_state)
srcs_cpl_only += \
        modules/audio_processing/aec3/echo_canceller3_config.cc \
        modules/audio_processing/aec3/aec3_common.cc \
        modules/audio_processing/aec3/fft_data_avx2.cc \
        modules/audio_processing/aec3/fft_buffer.cc \
        modules/audio_processing/aec3/aec3_fft.cc \
        modules/audio_processing/aec3/block_buffer.cc \
        modules/audio_processing/aec3/block_delay_buffer.cc \
        modules/audio_processing/aec3/render_buffer.cc \
        modules/audio_processing/aec3/render_delay_buffer.cc \
        modules/audio_processing/aec3/downsampled_render_buffer.cc \
        modules/audio_processing/aec3/spectrum_buffer.cc \
        modules/audio_processing/aec3/render_signal_analyzer.cc \
        modules/audio_processing/aec3/erl_estimator.cc \
        modules/audio_processing/aec3/fullband_erle_estimator.cc \
        modules/audio_processing/aec3/signal_dependent_erle_estimator.cc \
        modules/audio_processing/aec3/subband_erle_estimator.cc \
        modules/audio_processing/aec3/erle_estimator.cc \
        modules/audio_processing/aec3/stationarity_estimator.cc \
        modules/audio_processing/aec3/echo_audibility.cc \
        modules/audio_processing/aec3/filter_analyzer.cc \
        modules/audio_processing/aec3/transparent_mode.cc \
        modules/audio_processing/aec3/aec_state.cc
# aec3(echo_remover: suppressor, subtractor, ...)
srcs_cpl_only += \
        modules/audio_processing/aec3/dominant_nearend_detector.cc \
        modules/audio_processing/aec3/subband_nearend_detector.cc \
        modules/audio_processing/aec3/moving_average.cc \
        modules/audio_processing/aec3/vector_math_avx2.cc \
        modules/audio_processing/aec3/suppression_filter.cc \
        modules/audio_processing/aec3/suppression_gain_limiter.cc \
        modules/audio_processing/aec3/reverb_model.cc \
        modules/audio_processing/aec3/adaptive_fir_filter.cc \
        modules/audio_processing/aec3/adaptive_fir_filter_avx2.cc \
        modules/audio_processing/aec3/adaptive_fir_filter_erl.cc \
        modules/audio_processing/aec3/adaptive_fir_filter_erl_avx2.cc \
        modules/audio_processing/aec3/reverb_model_fallback.cc \
        modules/audio_processing/aec3/reverb_model_estimator.cc \
        modules/audio_processing/aec3/reverb_decay_estimator.cc \
        modules/audio_processing/aec3/reverb_frequency_response.cc \
        modules/audio_processing/aec3/subtractor_output_analyzer.cc \
        modules/audio_processing/aec3/subtractor_output.cc \
        modules/audio_processing/aec3/refined_filter_update_gain.cc \
        modules/audio_processing/aec3/coarse_filter_update_gain.cc \
        modules/audio_processing/aec3/echo_path_variability.cc \
        modules/audio_processing/aec3/subtractor.cc \
        modules/audio_processing/aec3/suppression_gain.cc \
        modules/audio_processing/aec3/residual_echo_estimator.cc \
        modules/audio_processing/aec3/comfort_noise_generator.cc \
        modules/audio_processing/aec3/echo_remover_metrics.cc \
        modules/audio_processing/aec3/echo_remover.cc
# aec3(block_processor: echo_remover, render_delay_controller, )
srcs_cpl_only += \
        modules/audio_processing/aec3/matched_filter_avx2.cc \
        modules/audio_processing/aec3/matched_filter.cc \
        modules/audio_processing/aec3/matched_filter_lag_aggregator.cc \
        modules/audio_processing/aec3/alignment_mixer.cc \
        modules/audio_processing/aec3/clockdrift_detector.cc \
        modules/audio_processing/aec3/decimator.cc \
        modules/audio_processing/aec3/echo_path_delay_estimator.cc \
        modules/audio_processing/aec3/render_delay_controller_metrics.cc \
        modules/audio_processing/aec3/render_delay_controller.cc \
        modules/audio_processing/aec3/block_processor_metrics.cc \
        modules/audio_processing/aec3/block_processor.cc \
        modules/audio_processing/aec3/block_framer.cc \
        modules/audio_processing/aec3/frame_blocker.cc \
        modules/audio_processing/aec3/api_call_jitter_metrics.cc \
        modules/audio_processing/aec3/echo_canceller3.cc \
        modules/audio_processing/aec3/echo_canceller3_factory.cc
srcs_cpl_only += \
        modules/audio_processing/aec3/matrix_buffer.cc \
        modules/audio_processing/aec3/vector_buffer.cc \
        modules/audio_processing/aec3/skew_estimator.cc \
        modules/audio_processing/aec3/shadow_filter_update_gain.cc \
        modules/audio_processing/aec3/main_filter_update_gain.cc
# vad
srcs += \
        modules/audio_processing/vad/gmm.cc \
        modules/audio_processing/vad/pitch_based_vad.cc \
        modules/audio_processing/vad/pitch_internal.cc \
        modules/audio_processing/vad/pole_zero_filter.cc \
        modules/audio_processing/vad/standalone_vad.cc \
        modules/audio_processing/vad/vad_audio_proc.cc \
        modules/audio_processing/vad/vad_circular_buffer.cc \
        modules/audio_processing/vad/voice_activity_detector.cc \
        modules/audio_processing/vad/voice_detection_impl.cc
# vad(isac pitch_based_vad)
srcs_c += \
        modules/third_party/fft/fft.c \
        modules/audio_coding/codecs/isac/main/source/arith_routines.c \
        modules/audio_coding/codecs/isac/main/source/arith_routines_hist.c \
        modules/audio_coding/codecs/isac/main/source/arith_routines_logist.c \
        modules/audio_coding/codecs/isac/main/source/bandwidth_estimator.c \
        modules/audio_coding/codecs/isac/main/source/crc.c \
        modules/audio_coding/codecs/isac/main/source/decode_bwe.c \
        modules/audio_coding/codecs/isac/main/source/decode.c \
        modules/audio_coding/codecs/isac/main/source/encode.c \
        modules/audio_coding/codecs/isac/main/source/encode_lpc_swb.c \
        modules/audio_coding/codecs/isac/main/source/entropy_coding.c \
        modules/audio_coding/codecs/isac/main/source/filterbanks.c \
        modules/audio_coding/codecs/isac/main/source/filter_functions.c \
        modules/audio_coding/codecs/isac/main/source/intialize.c \
        modules/audio_coding/codecs/isac/main/source/isac.c \
        modules/audio_coding/codecs/isac/main/source/isac_vad.c \
        modules/audio_coding/codecs/isac/main/source/lattice.c \
        modules/audio_coding/codecs/isac/main/source/lpc_analysis.c \
        modules/audio_coding/codecs/isac/main/source/lpc_gain_swb_tables.c \
        modules/audio_coding/codecs/isac/main/source/lpc_shape_swb12_tables.c \
        modules/audio_coding/codecs/isac/main/source/lpc_shape_swb16_tables.c \
        modules/audio_coding/codecs/isac/main/source/lpc_tables.c \
        modules/audio_coding/codecs/isac/main/source/pitch_estimator.c \
        modules/audio_coding/codecs/isac/main/source/pitch_filter.c \
        modules/audio_coding/codecs/isac/main/source/pitch_gain_tables.c \
        modules/audio_coding/codecs/isac/main/source/pitch_lag_tables.c \
        modules/audio_coding/codecs/isac/main/source/spectrum_ar_model_tables.c \
        modules/audio_coding/codecs/isac/main/source/transform.c
# beamformer
srcs += \
        modules/audio_processing/beamformer/array_util.cc \
        modules/audio_processing/beamformer/covariance_matrix_generator.cc \
        modules/audio_processing/beamformer/nonlinear_beamformer.cc
# intelligibility
srcs += \
        modules/audio_processing/intelligibility/intelligibility_enhancer.cc \
        modules/audio_processing/intelligibility/intelligibility_utils.cc
# transient
srcs += \
        modules/audio_processing/transient/moving_moments.cc \
        modules/audio_processing/transient/transient_detector.cc \
        modules/audio_processing/transient/transient_suppressor.cc \
        modules/audio_processing/transient/wpd_node.cc \
        modules/audio_processing/transient/wpd_tree.cc
# apm
# srcs += \
        modules/audio_processing/echo_control_mobile_proxy.cc \
        modules/audio_processing/echo_control_proxy.cc
# conference_mixer
srcs += \
        modules/audio_conference_mixer/source/audio_conference_mixer_impl.cc \
        modules/audio_conference_mixer/source/audio_frame_manipulator.cc \
        modules/audio_conference_mixer/source/time_scheduler.cc
# mixer
srcs_cpl_only += \
        modules/audio_mixer/audio_frame_manipulator.cc \
        modules/audio_mixer/audio_mixer_impl.cc \
        modules/audio_mixer/channel_mixer.cc \
        modules/audio_mixer/channel_mixing_matrix.cc \
        modules/audio_mixer/default_output_rate_calculator.cc \
        modules/audio_mixer/frame_combiner.cc \
        modules/audio_mixer/gain_change_calculator.cc \
        modules/audio_mixer/sine_wave_generator.cc
# neteq
srcs += \
        modules/audio_coding/neteq/dsp_helper.cc \
        modules/audio_coding/neteq/audio_multi_vector.cc \
        modules/audio_coding/neteq/audio_vector.cc \
        modules/audio_coding/neteq/cross_correlation.cc \
        modules/audio_coding/neteq/time_stretch.cc \
        modules/audio_coding/neteq/preemptive_expand.cc \
        modules/audio_coding/neteq/accelerate.cc \
        modules/audio_coding/neteq/tick_timer.cc \
        modules/audio_coding/neteq/delay_peak_detector.cc \
        modules/audio_coding/neteq/histogram.cc \
        modules/audio_coding/neteq/delay_manager.cc \
        modules/audio_coding/neteq/buffer_level_filter.cc \
        modules/audio_coding/neteq/normal.cc
srcs_cpl_only += \
        modules/audio_coding/codecs/cng/webrtc_cng.cc \
        modules/audio_coding/neteq/background_noise.cc \
        modules/audio_coding/neteq/post_decode_vad.cc \
        modules/audio_coding/neteq/comfort_noise.cc \
        modules/audio_coding/neteq/packet.cc \
        modules/audio_coding/neteq/merge.cc \
        modules/audio_coding/neteq/statistics_calculator.cc \
        modules/audio_coding/neteq/expand.cc \
        modules/audio_coding/neteq/expand_uma_logger.cc \
        modules/audio_coding/neteq/dtmf_buffer.cc \
        modules/audio_coding/neteq/dtmf_tone_generator.cc \
        modules/audio_coding/neteq/sync_buffer.cc \
        modules/audio_coding/neteq/random_vector.cc \
        modules/audio_coding/neteq/nack_tracker.cc \
        modules/audio_coding/neteq/red_payload_splitter.cc \
        modules/audio_coding/neteq/timestamp_scaler.cc \
        modules/audio_coding/neteq/packet_buffer.cc
srcs_cpl_only += \
        modules/audio_coding/neteq/neteq_decoder_enum.cc \
        modules/audio_coding/neteq/audio_decoder_impl.cc \
        # modules/audio_coding/neteq/decoder_database_impl.cc \
        # modules/audio_coding/neteq/decision_logic.cc \
        modules/audio_coding/neteq/decision_logic_fax.cc \
        modules/audio_coding/neteq/decision_logic_normal.cc \
        modules/audio_coding/neteq/neteq.cc \
        modules/audio_coding/neteq/neteq_impl.cc \
        # modules/audio_coding/neteq/rtcp.cc
