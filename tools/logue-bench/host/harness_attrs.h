/*
 * Mach-O safe replacements for common/attributes.h.
 *
 * The SDK marks the unit header with section(".unit_header"), which is an
 * ELF-style specifier that Mach-O rejects ("requires a segment and section
 * separated by a comma"). The harness compiles the unmodified project
 * sources with -DATTRIBUTES_H_=1, which makes the SDK's attributes.h a
 * no-op, and force-includes this file in its place.
 *
 * Nothing here is used for the device build.
 */
#ifndef HARNESS_ATTRS_H_
#define HARNESS_ATTRS_H_

#define noopt
#define fast
#define force_inline inline __attribute__((always_inline))
#define fast_inline inline __attribute__((always_inline))

#define __unit_callback __attribute__((used))
#define __unit_header __attribute__((used))

#endif // HARNESS_ATTRS_H_
