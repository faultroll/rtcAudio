srcs_1 += \
        common_audio/audio_converter.cc \
        common_audio/audio_ring_buffer.cc \
        common_audio/audio_util.cc \
        common_audio/blocker.cc \
        common_audio/channel_buffer.cc \
        common_audio/fir_filter_avx2.cc \
        common_audio/fir_filter_c.cc \
        common_audio/fir_filter_factory.cc \
        common_audio/fir_filter_neon.cc \
        common_audio/fir_filter_sse.cc \
        common_audio/lapped_transform.cc \
        common_audio/real_fourier.cc \
        common_audio/real_fourier_ooura.cc \
        common_audio/ring_buffer.c \
        common_audio/smoothing_filter.cc \
        common_audio/sparse_fir_filter.cc \
        common_audio/window_generator.cc \
srcs_2 += \
        # common_audio/wav_file.cc \
        common_audio/wav_header.cc
srcs_1 += \
        common_audio/resampler/push_resampler.cc \
        common_audio/resampler/push_sinc_resampler.cc \
        common_audio/resampler/resampler.cc \
        common_audio/resampler/sinc_resampler_avx2.cc \
        common_audio/resampler/sinc_resampler.cc \
        common_audio/resampler/sinc_resampler_neon.cc \
        common_audio/resampler/sinc_resampler_sse.cc \
        common_audio/resampler/sinusoidal_linear_chirp_source.cc
srcs_1 += \
        common_audio/third_party/ooura/fft_size_128/ooura_fft.cc \
        common_audio/third_party/ooura/fft_size_128/ooura_fft.h \
        common_audio/third_party/ooura/fft_size_128/ooura_fft_mips.cc \
        common_audio/third_party/ooura/fft_size_128/ooura_fft_neon.cc \
        common_audio/third_party/ooura/fft_size_128/ooura_fft_sse2.cc \
        common_audio/third_party/ooura/fft_size_128/ooura_fft_tables_common.h \
        common_audio/third_party/ooura/fft_size_128/ooura_fft_tables_neon_sse2.h \
        common_audio/third_party/ooura/fft_size_256/fft4g.cc \
        common_audio/third_party/ooura/fft_size_256/fft4g.h \
        common_audio/third_party/spl_sqrt_floor/spl_sqrt_floor.c \
        common_audio/third_party/spl_sqrt_floor/spl_sqrt_floor_mips.c \
        common_audio/third_party/spl_sqrt_floor/spl_sqrt_floor_arm.S
srcs_1 += \
        common_audio/signal_processing/auto_correlation.c \
        common_audio/signal_processing/auto_corr_to_refl_coef.c \
        common_audio/signal_processing/complex_bit_reverse.c \
        common_audio/signal_processing/complex_bit_reverse_mips.c \
        common_audio/signal_processing/complex_fft.c \
        common_audio/signal_processing/complex_fft_mips.c \
        common_audio/signal_processing/copy_set_operations.c \
        common_audio/signal_processing/cross_correlation.c \
        common_audio/signal_processing/cross_correlation_mips.c \
        common_audio/signal_processing/cross_correlation_neon.c \
        common_audio/signal_processing/division_operations.c \
        common_audio/signal_processing/downsample_fast.c \
        common_audio/signal_processing/downsample_fast_mips.c \
        common_audio/signal_processing/downsample_fast_neon.c \
        common_audio/signal_processing/energy.c \
        common_audio/signal_processing/filter_ar.c \
        common_audio/signal_processing/filter_ar_fast_q12.c \
        common_audio/signal_processing/filter_ar_fast_q12_mips.c \
        common_audio/signal_processing/filter_ma_fast_q12.c \
        common_audio/signal_processing/get_hanning_window.c \
        common_audio/signal_processing/get_scaling_square.c \
        common_audio/signal_processing/ilbc_specific_functions.c \
        common_audio/signal_processing/levinson_durbin.c \
        common_audio/signal_processing/lpc_to_refl_coef.c \
        common_audio/signal_processing/min_max_operations.c \
        common_audio/signal_processing/min_max_operations_mips.c \
        common_audio/signal_processing/min_max_operations_neon.c \
        common_audio/signal_processing/randomization_functions.c \
        common_audio/signal_processing/real_fft.c \
        common_audio/signal_processing/refl_coef_to_lpc.c \
        common_audio/signal_processing/resample_48khz.c \
        common_audio/signal_processing/resample_by_2.c \
        common_audio/signal_processing/resample_by_2_internal.c \
        common_audio/signal_processing/resample_by_2_mips.c \
        common_audio/signal_processing/resample.c \
        common_audio/signal_processing/resample_fractional.c \
        common_audio/signal_processing/spl_init.c \
        common_audio/signal_processing/spl_inl.c \
        common_audio/signal_processing/splitting_filter.c \
        common_audio/signal_processing/spl_sqrt.c \
        common_audio/signal_processing/sqrt_of_one_minus_x_squared.c \
        common_audio/signal_processing/vector_scaling_operations.c \
        common_audio/signal_processing/vector_scaling_operations_mips.c \
        common_audio/signal_processing/complex_bit_reverse_arm.S \
        common_audio/signal_processing/filter_ar_fast_q12_armv7.S \
        common_audio/signal_processing/dot_product_with_scale.c \
        # common_audio/signal_processing/dot_product_with_scale.cc
srcs_1 += \
        common_audio/vad/vad.cc \
        common_audio/vad/vad_core.c \
        common_audio/vad/vad_filterbank.c \
        common_audio/vad/vad_gmm.c \
        common_audio/vad/vad_sp.c \
        common_audio/vad/webrtc_vad.c
