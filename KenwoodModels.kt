/*
 * libkenwoodk - Kenwood CAT driver for the iSDR driver host
 *
 * Copyright (C) 2026 Isak Ruas <isakruas@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */
package com.isaklab.libkenwoodk

/**
 * What each Kenwood `ID` code is known to be, and what its bandscope streams.
 *
 * Two tiers exist. The scope tier (TS-890S, TS-990S) speaks the surface this
 * dialect implements: `OM` modes, `##`-prefixed KNS LAN commands and the
 * `DD`/`BS` bandscope family. The classic tier (TS-480, TS-590, TS-2000, ...)
 * speaks the older `MD`/`IF` serial dialect that this module deliberately
 * does not half-implement: connect refuses with a status that names what is
 * unsupported, keeping the capability surface honest.
 */
object KenwoodModels {

    /** Which `##ID` login encoding a rig's KNS server expects. */
    enum class LoginShape {
        /** `##ID<type:1><acctlen:2><pwlen:2>...` (TS-890S). */
        TS890,

        /** `##ID<acctlen:1><pwlen:1>...`, fields 1-8 chars (TS-990S). */
        TS990,
    }

    /** A rig whose bandscope this dialect can stream. */
    data class ScopeModel(
        val name: String,
        val login: LoginShape,
        /**
         * Highest legal `DD0` streaming code (TS-890S: 5; TS-990S: 2 — its
         * serial split modes do not exist).
         */
        val dd0Max: Int,
        /**
         * Bins per main-scope line; both models stream 640 over the derived
         * span. The TS-990S also has a sub-scope plane (`##DD3`, 285 bins) —
         * out of scope here, the main scope is the spectrum source.
         */
        val bins: Int,
    )

    /** `ID` answer codes. */
    const val ID_TS890S = 24
    const val ID_TS990S = 23

    /**
     * Scope-tier model for an `ID` code; null means classic tier (or
     * unknown), which this dialect refuses rather than half-drives.
     */
    fun scopeModel(idCode: Int): ScopeModel? = when (idCode) {
        ID_TS890S -> ScopeModel("TS-890S", LoginShape.TS890, 5, KenwoodProtocol.SCOPE_BINS)
        ID_TS990S -> ScopeModel("TS-990S", LoginShape.TS990, 2, KenwoodProtocol.SCOPE_BINS)
        else -> null
    }

    /** Human name for a classic-tier `ID` code, for the refusal status. */
    fun classicName(idCode: Int): String = when (idCode) {
        19 -> "TS-2000"
        21 -> "TS-480/TS-590"
        else -> "Kenwood ID %03d".format(idCode)
    }
}
