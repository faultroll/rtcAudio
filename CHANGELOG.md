
0. uses Makefile
1. remove audio&api
    - audio_frame.h & audio_frame_operations.h moved to modules/include (like m59 modules/interface)
    - aec3 headers in api/audio moved to modules/audio_processing/aec3 (each module have its own interface)
2. rtc_base using m88
    - only use c11 & c++11
    - absl removed (rtc::Optional instead)
3. remove system_wrappers
    - only metrics needed
4. add post processing
    - mixer/conference_mixer
    - neteq
5. split modules and add demos for each module
    - xxx_impl to xxx dir
    - xxx interface to include dir
6. rename modules compact with older versions
    - level_controllor in m59 is agc2 in m70
    - ns in m88 renamed to ns2
    - using m88 |agc_manager_direct| instead of current
7. remove =default, =delete

TODOs
1. remove aligned_array (lapped_transform uses this)
2. change array_view (many use) to pure c
4. use constructor_magic, remove = default
5. use WEBRTC_TRACE instead of LOG_XXX (more c-like)
6. remove CritScope in apm submodules(aec/agc/...) like m88 (thread safe is controlled by user(like apm), just algorithm)
