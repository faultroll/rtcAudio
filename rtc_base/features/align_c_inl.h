
#if defined(_ALIGN_C_USE_STD)
// Nothing here
#else 
// #include "align_c.h"

// The functions declared here
// 1) Allocates block of aligned memory.
// 2) Re-calculates a pointer such that it is aligned to a higher or equal
//    address.
// Note: alignment must be a power of two. The alignment is in bytes.

#include <stddef.h>
#include <string.h>  // for memcpy
#include <stdlib.h> // for malloc, free

#if defined(_ALIGN_C_USE_POSIX)
    #include <stdint.h>
#elif defined(_ALIGN_C_USE_WIN)
    #include <windows.h>
#elif defined(_ALIGN_C_USE_NONE)
    #error "align_c: current not support |_ALIGN_C_USE_NONE|"
#else
    #error "align_c: unknown branch in |aligned_malloc|"
#endif /* defined(_ALIGN_C_USE_POSIX) */

// Reference on memory alignment:
// http://stackoverflow.com/questions/227897/solve-the-memory-alignment-in-c-interview-question-that-stumped-me

// Returns a pointer to the first boundry of |alignment| bytes following the
// address of |ptr|.
// Note that there is no guarantee that the memory in question is available.
// |ptr| has no requirements other than it can't be NULL.
/* uintptr_t alignup(uintptr_t start_pos, size_t alignment)
{
    // The pointer should be aligned with |alignment| bytes. The - 1 guarantees
    // that it is aligned towards the closest higher (right) address.
    return (start_pos + alignment - 1) & ~(alignment - 1);
} */

// Alignment must be an integer power of two.
/* bool alignment_valid(size_t alignment)
{
    if (!alignment) {
        return false;
    }
    return (alignment & (alignment - 1)) == 0;
} */

/* void *alignup(const void *pointer, size_t alignment)
{
    if (!pointer) {
        return NULL;
    }
    if (!alignment_valid(alignment)) {
        return NULL;
    }
    uintptr_t start_pos = (uintptr_t)pointer;
    return (void *)alignup(start_pos, alignment);
} */

// Allocates memory of |size| bytes aligned on an |alignment| boundry.
// The return value is a pointer to the memory. Note that the memory must
// be de-allocated using AlignedFree.
static inline
void *aligned_alloc(size_t size, size_t alignment)
{
    if (size == 0) {
        return NULL;
    }
    if (!/* alignment_valid */ALIGN_VALID(alignment)) {
        return NULL;
    }

    // The memory is aligned towards the lowest address that so only
    // alignment - 1 bytes needs to be allocated.
    // A pointer to the start of the memory must be stored so that it can be
    // retreived for deletion, ergo the sizeof(uintptr_t).
    void *memory_pointer = malloc(size + sizeof(uintptr_t) + alignment - 1);
    if (memory_pointer == NULL) {
        return NULL;
    } // RTC_CHECK(memory_pointer) /* << "Couldn't allocate memory in AlignedMalloc" */;

    // Aligning after the sizeof(uintptr_t) bytes will leave room for the header
    // in the same memory block.
    uintptr_t align_start_pos = (uintptr_t)memory_pointer;
    align_start_pos += sizeof(uintptr_t);
    uintptr_t aligned_pos = /* alignup */ALIGN_UP(align_start_pos, alignment);
    void *aligned_pointer = (void *)aligned_pos;

    // Store the address to the beginning of the memory just before the aligned
    // memory.
    uintptr_t header_pos = aligned_pos - sizeof(uintptr_t);
    void *header_pointer = (void *)header_pos;
    uintptr_t memory_start = (uintptr_t)memory_pointer;
    memcpy(header_pointer, &memory_start, sizeof(uintptr_t));

    return aligned_pointer;
}

// De-allocates memory created using the aligned_alloc() API.
static inline
void aligned_free(void *mem_block)
{
    if (mem_block == NULL) {
        return;
    }
    uintptr_t aligned_pos = (uintptr_t)mem_block;
    uintptr_t header_pos = aligned_pos - sizeof(uintptr_t);

    // Read out the address of the AlignedMemory struct from the header.
    uintptr_t memory_start_pos = *(uintptr_t *)header_pos;
    void *memory_start = (void *)memory_start_pos;
    free(memory_start);
}
#endif /* defined(_ALIGN_C_USE_STD) */
