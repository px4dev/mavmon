/*
 * Copyright (c) 2012, px4dev, <px4@purgatory.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * o Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file ui_menu.h
 *
 * Menu tree for MAVmon.
 *
 * Font usage:
 * 0 - normal text (default)
 * 1 - titles
 * 2 - m2 glyphs (should already be set up?)
 */

#include <m2.h>

#define PRIVATE static const

#pragma GCC diagnostic ignored "-Wold-style-declaration"
const M2_EXTERN_ALIGN(_top);
#pragma GCC diagnostic warning "-Wold-style-declaration"

/* default back-to-top control */
PRIVATE M2_ROOT (_top_return,		"f2",	"\x43",		&_top);

PRIVATE M2_SPACE(_pad, "w8h1");

/*
 * The top menu of a menu tree.
 */
#define TOP_MENU(__name, __title, __elts...)				\
	static const M2_LABEL(__name##_title, "f1", __title);		\
	static M2_LIST(__name##_list) = { &__name##_title, __elts };	\
	static const M2_VLIST(__name##_vlist, NULL, __name##_list);	\
	const M2_ALIGN(__name, "-0|2W64H63", &__name##_vlist)


/*
 * A sub-menu that returns to the top menu.
 */
#define MENU(__name, __title, __elts...)					\
	static const M2_LABEL(__name##_title, "f1", __title);			\
	static M2_LIST(__name##_header_list) = {&_top_return, &_pad, &__name##_title}; \
	static const M2_HLIST(__name##_header, NULL, __name##_header_list);	\
	static M2_LIST(__name##_list) = { &__name##_header, __elts };		\
	static const M2_VLIST(__name##_vlist, NULL, __name##_list);		\
	static const M2_ALIGN(__name, "-0|2W64H63", &__name##_vlist)


/*
 * Settings menu
 */
uint32_t value;
PRIVATE M2_U32NUM(_settings_number1,	"a0",	&value);
PRIVATE M2_U32NUM(_settings_number2,	"a0",	&value);
PRIVATE M2_LABEL(_settings_dummy,	NULL,	"this is a placeholder");

MENU(_settings, "Settings",
	&_settings_number1,
	&_settings_number2,
	&_settings_dummy);

/*
 * Radio setup menu
 */
PRIVATE M2_LABEL(_radio_dummy,		NULL,	"this is a placeholder");

MENU(_radio, "Radio Setup",
	&_radio_dummy);

/*
 * Empty page with just a back widget.
 *
 * Suitable for e.g. rendering stuff.
 */
static M2_LIST(_empty_list) = {&_top_return};
static const M2_VLIST(_empty_vlist, NULL, _empty_list);
static const M2_ALIGN(_empty, "-0|2W64H63", &_empty_vlist);

/*
 * Top-level menu.
 */
PRIVATE M2_ROOT (_top_settings,		NULL,	"SETTINGS...",		&_settings);
PRIVATE M2_ROOT (_top_radio,		NULL,	"RADIO SETUP...",	&_radio);
PRIVATE M2_ROOT (_top_empty,		NULL,	"EMPTY...",		&_empty);

TOP_MENU(_top, "MAVmon",
	&_top_settings,
	&_top_empty,
	&_top_radio);

