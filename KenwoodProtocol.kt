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

import java.nio.ByteBuffer
import java.nio.charset.CharacterCodingException

/**
 * Frame codec for the Kenwood CAT dialect (TS-890S / TS-990S).
 *
 * Builds controller-to-rig command strings and parses rig-to-controller
 * answers. Holds no I/O and no clock; the caller owns the port, the retry
 * policy and the scope-line assembly state, so every byte of the wire format
 * is unit-testable without a rig.
 *
 * Wire format: ASCII commands terminated by `;`, no CR/LF. Segments may
 * concatenate on the wire; the [Splitter] cuts on `;`. The rig answers an
 * unacceptable command with the bare error token `?;`. Set commands produce
 * no acknowledge of their own — with `AI2` on, the resulting state change
 * comes back shaped exactly like an Answer, and the caller confirms a set by
 * reading the value back.
 *
 * LAN (`##`-prefixed) commands exist only on the KNS TCP connection.
 */
object KenwoodProtocol {

    // ---- numeric field helpers ----------------------------------------------

    /** Highest frequency expressible in the 11-digit Hz field. */
    const val MAX_FREQ_HZ = 99_999_999_999L

    /** True when [s] is exactly [len] ASCII decimal digits. */
    private fun digits(s: String, len: Int): Boolean =
        s.length == len && s.all { it in '0'..'9' }

    /** Parse an exact-width zero-padded decimal field. */
    private fun parseDigits(s: String, len: Int): Long? =
        if (digits(s, len)) s.toLongOrNull() else null

    private fun String.afterPrefix(prefix: String): String? =
        if (startsWith(prefix)) substring(prefix.length) else null

    // ---- operating modes (OM) -----------------------------------------------

    /**
     * OM mode codes, one hexadecimal digit on the wire. `0` and `8` are not
     * assigned and are rejected on both build and parse.
     */
    const val MODE_LSB = 0x1
    const val MODE_USB = 0x2
    const val MODE_CW = 0x3
    const val MODE_FM = 0x4
    const val MODE_AM = 0x5
    const val MODE_FSK = 0x6
    const val MODE_CW_R = 0x7
    const val MODE_FSK_R = 0x9
    const val MODE_PSK = 0xA
    const val MODE_PSK_R = 0xB
    const val MODE_LSB_D = 0xC
    const val MODE_USB_D = 0xD
    const val MODE_FM_D = 0xE
    const val MODE_AM_D = 0xF

    /** True for a mode code the OM table assigns. */
    fun modeValid(mode: Int): Boolean = mode in 0x1..0xF && mode != 0x8

    private fun modeDigit(mode: Int): Char? =
        if (modeValid(mode)) mode.toString(16).uppercase()[0] else null

    private fun modeFromDigit(c: Char): Int? {
        val v = Character.digit(c, 16)
        return if (v >= 0 && modeValid(v)) v else null
    }

    /** Human name of an OM mode code. */
    fun modeName(mode: Int): String? = when (mode) {
        MODE_LSB -> "LSB"
        MODE_USB -> "USB"
        MODE_CW -> "CW"
        MODE_FM -> "FM"
        MODE_AM -> "AM"
        MODE_FSK -> "FSK"
        MODE_CW_R -> "CW-R"
        MODE_FSK_R -> "FSK-R"
        MODE_PSK -> "PSK"
        MODE_PSK_R -> "PSK-R"
        MODE_LSB_D -> "LSB-D"
        MODE_USB_D -> "USB-D"
        MODE_FM_D -> "FM-D"
        MODE_AM_D -> "AM-D"
        else -> null
    }

    // ---- command builders ----------------------------------------------------

    fun getFa(): String = "FA;"

    fun getFb(): String = "FB;"

    /** VFO A frequency set, 11 digits zero-padded Hz. Null outside the field. */
    fun setFa(hz: Long): String? =
        if (hz in 0..MAX_FREQ_HZ) "FA%011d;".format(hz) else null

    fun setFb(hz: Long): String? =
        if (hz in 0..MAX_FREQ_HZ) "FB%011d;".format(hz) else null

    /** Parse `FAxxxxxxxxxxx` / `FBxxxxxxxxxxx` answers (strict 11-digit width). */
    fun parseFa(seg: String): Long? = seg.afterPrefix("FA")?.let { parseDigits(it, 11) }

    fun parseFb(seg: String): Long? = seg.afterPrefix("FB")?.let { parseDigits(it, 11) }

    /** Operating-mode read of display area [receiver] (0 left, 1 right). */
    fun getOm(receiver: Int): String? = if (receiver in 0..1) "OM$receiver;" else null

    /**
     * Operating-mode set. The receiver digit is carried but ignored by the
     * rig on set; the mode is one hex digit from the OM table.
     */
    fun setOm(receiver: Int, mode: Int): String? {
        if (receiver !in 0..1) return null
        val d = modeDigit(mode) ?: return null
        return "OM$receiver$d;"
    }

    /** Parse an `OM` answer: (receiver, mode code). */
    fun parseOm(seg: String): Pair<Int, Int>? {
        val rest = seg.afterPrefix("OM") ?: return null
        if (rest.length != 2) return null
        val receiver = Character.digit(rest[0], 10)
        if (receiver !in 0..1) return null
        val mode = modeFromDigit(rest[1]) ?: return null
        return Pair(receiver, mode)
    }

    /** Keying: 0 SEND, 1 DATA SEND, 2 tune. */
    fun setTx(kind: Int): String? = if (kind in 0..2) "TX$kind;" else null

    fun setRx(): String = "RX;"

    /**
     * True when the segment reports transmit (`TX`, optionally with the
     * keying digit); false for the `RX` report.
     */
    fun parseTxState(seg: String): Boolean? = when {
        seg == "RX" -> false
        seg == "TX" -> true
        else -> {
            val d = seg.afterPrefix("TX")
            if (d != null && d.length == 1 && d[0] in '0'..'2') true else null
        }
    }

    fun getFr(): String = "FR;"

    fun getFt(): String = "FT;"

    /** Receive VFO select (0 VFO A, 1 VFO B, 3 memory). */
    fun setFr(v: Int): String? = if (v == 0 || v == 1 || v == 3) "FR$v;" else null

    /** Transmit VFO select; split when FR differs from FT. */
    fun setFt(v: Int): String? = if (v == 0 || v == 1 || v == 3) "FT$v;" else null

    fun parseFr(seg: String): Int? {
        val v = seg.afterPrefix("FR")?.let { parseDigits(it, 1) }?.toInt() ?: return null
        return if (v == 0 || v == 1 || v == 3) v else null
    }

    fun parseFt(seg: String): Int? {
        val v = seg.afterPrefix("FT")?.let { parseDigits(it, 1) }?.toInt() ?: return null
        return if (v == 0 || v == 1 || v == 3) v else null
    }

    fun getId(): String = "ID;"

    /** Parse the 3-digit `ID` answer (`ID024` = 24). */
    fun parseId(seg: String): Int? =
        seg.afterPrefix("ID")?.let { parseDigits(it, 3) }?.toInt()

    /**
     * Auto-information ON (volatile, per-connector; required for scope
     * streaming).
     */
    fun setAi2(): String = "AI2;"

    fun getSm(): String = "SM;"

    /** Parse the S-meter answer: 4 digits, 0000-0070 dots. */
    fun parseSm(seg: String): Int? {
        val v = seg.afterPrefix("SM")?.let { parseDigits(it, 4) } ?: return null
        return if (v <= 70) v.toInt() else null
    }

    /** Enable ([on] = true) reporting of meter [meter] (single digit selector). */
    fun setRm(meter: Int, on: Boolean): String? =
        if (meter in 0..9) "RM$meter${if (on) 1 else 0};" else null

    /** Parse a meter report: (meter selector, 4-digit reading). */
    fun parseRm(seg: String): Pair<Int, Int>? {
        val rest = seg.afterPrefix("RM") ?: return null
        if (rest.length != 5) return null
        val meter = parseDigits(rest.substring(0, 1), 1)?.toInt() ?: return null
        val value = parseDigits(rest.substring(1), 4)?.toInt() ?: return null
        return Pair(meter, value)
    }

    /**
     * SWR meter calibration: piecewise-linear between the documented
     * breakpoints 0=1.0, 11=1.5, 23=2.0, 35=3.0, 70=infinity (readings at or
     * past the last breakpoint report infinity).
     */
    fun swrFromMeter(reading: Int): Float {
        val cal = floatArrayOf(0f, 1.0f, 11f, 1.5f, 23f, 2.0f, 35f, 3.0f)
        val r = reading.toFloat()
        if (r >= 70f) return Float.POSITIVE_INFINITY
        for (i in 0 until cal.size / 2 - 1) {
            val x0 = cal[2 * i]
            val y0 = cal[2 * i + 1]
            val x1 = cal[2 * i + 2]
            val y1 = cal[2 * i + 3]
            if (r <= x1) return y0 + (r - x0) / (x1 - x0) * (y1 - y0)
        }
        // Between the 3.0 breakpoint and the infinity endpoint there is no
        // finite slope; report the last calibrated value.
        return 3.0f
    }

    /** AF gain, 000-255, no receiver digit. */
    fun setAg(level: Int): String = "AG%03d;".format(level and 0xFF)

    fun getAg(): String = "AG;"

    fun setRg(level: Int): String = "RG%03d;".format(level and 0xFF)

    fun getRg(): String = "RG;"

    fun setSq(level: Int): String = "SQ%03d;".format(level and 0xFF)

    fun getSq(): String = "SQ;"

    /** Parse a 3-digit 000-255 level answer for the given 2-letter prefix. */
    fun parseLevel(seg: String, prefix: String): Int? {
        val v = seg.afterPrefix(prefix)?.let { parseDigits(it, 3) } ?: return null
        return if (v <= 255) v.toInt() else null
    }

    /** Power-status query, doubling as the LAN keepalive. */
    fun getPs(): String = "PS;"

    fun parsePs(seg: String): Boolean? =
        seg.afterPrefix("PS")?.let { parseDigits(it, 1) }?.let { it == 1L }

    // ---- segment splitter ----------------------------------------------------

    /**
     * Longest segment the splitter will hold: the LAN `##DD2` scope frame is
     * 1285 bytes before its terminator; anything much longer is a stream that
     * lost its `;`.
     */
    const val MAX_SEGMENT = 2048

    /** One `;`-terminated unit from the wire, terminator stripped. */
    sealed class Segment {
        /** A command/answer body, e.g. `FA00007074000`. */
        data class Cmd(val body: String) : Segment()

        /** The rig's `?;` error token: the previous command was unacceptable. */
        object Error : Segment()
    }

    /**
     * Accumulates raw bytes and yields complete `;`-terminated segments,
     * tolerating concatenation and arbitrary partial arrivals. Oversized or
     * non-text segments are dropped and counted rather than surfaced.
     */
    class Splitter {
        private var buf = ByteArray(64)
        private var len = 0

        /** An oversized segment is being skipped until its terminator. */
        private var discarding = false

        /** Segments discarded for being oversized or not text. */
        var dropped: Long = 0
            private set

        fun push(bytes: ByteArray, count: Int = bytes.size): List<Segment> {
            val out = ArrayList<Segment>()
            for (i in 0 until count) {
                val b = bytes[i]
                if (b == ';'.code.toByte()) {
                    val segLen = len
                    len = 0
                    if (discarding) {
                        discarding = false
                        continue
                    }
                    val s = decodeUtf8(buf, segLen)
                    when {
                        s == null -> dropped++
                        s == "?" -> out.add(Segment.Error)
                        s.isNotEmpty() -> out.add(Segment.Cmd(s))
                    }
                } else if (!discarding) {
                    if (len == buf.size) buf = buf.copyOf(buf.size * 2)
                    buf[len++] = b
                    if (len > MAX_SEGMENT) {
                        len = 0
                        discarding = true
                        dropped++
                    }
                }
            }
            return out
        }

        /** Strict decode: null for byte sequences that are not text. */
        private fun decodeUtf8(bytes: ByteArray, count: Int): String? = try {
            Charsets.UTF_8.newDecoder()
                .decode(ByteBuffer.wrap(bytes, 0, count))
                .toString()
        } catch (_: CharacterCodingException) {
            null
        }
    }

    // ---- KNS LAN handshake ---------------------------------------------------

    /** Connection request, the first bytes on a KNS TCP session. */
    fun knsConnect(): String = "##CN;"

    /**
     * True when the string is a printable single-line credential field with
     * no terminator inside it.
     */
    private fun credentialOk(s: String, max: Int): Boolean =
        s.isNotEmpty() && s.length <= max &&
            s.all { it.code in 0x21..0x7E && it != ';' }

    /**
     * TS-890S login: `##ID<type:1><acctlen:2><pwlen:2><account><password>;`,
     * type 0 admin / 1 user. Null when a field is empty, too long for its
     * 2-digit length, or not printable ASCII.
     */
    fun knsLoginTs890(admin: Boolean, account: String, password: String): String? {
        if (!credentialOk(account, 99) || !credentialOk(password, 99)) return null
        return "##ID%d%02d%02d%s%s;".format(
            if (admin) 0 else 1, account.length, password.length, account, password,
        )
    }

    /**
     * TS-990S login: `##ID<acctlen:1><pwlen:1><account><password>;`, both
     * fields 1-8 characters, no type digit.
     */
    fun knsLoginTs990(account: String, password: String): String? {
        if (!credentialOk(account, 8) || !credentialOk(password, 8)) return null
        return "##ID${account.length}${password.length}$account$password;"
    }

    /** `##CN1` granted / `##CN0` denied-or-busy. */
    fun parseKnsConnectReply(seg: String): Boolean? = when (seg) {
        "##CN1" -> true
        "##CN0" -> false
        else -> null
    }

    /** `##ID1` login accepted / `##ID0` rejected. */
    fun parseKnsLoginReply(seg: String): Boolean? = when (seg) {
        "##ID1" -> true
        "##ID0" -> false
        else -> null
    }

    /** Unsolicited `##UE<n>` user-event report. */
    fun parseKnsUe(seg: String): Int? {
        val d = seg.afterPrefix("##UE") ?: return null
        if (d.isEmpty() || d.length > 9 || !d.all { it in '0'..'9' }) return null
        return d.toIntOrNull()
    }

    /** `##TI<n>`: 1 = this connection is authorized to transmit. */
    fun parseKnsTi(seg: String): Boolean? =
        seg.afterPrefix("##TI")?.let { parseDigits(it, 1) }?.let { it == 1L }

    // ---- bandscope -----------------------------------------------------------

    /** Bins in one complete bandscope line. */
    const val SCOPE_BINS = 640

    /** Raw bin value of the display floor; higher values clamp here. */
    const val SCOPE_LEVEL_FLOOR = 0x8C

    /**
     * Scope streaming control: 0 off, 1/2/3 LAN high/medium/low update cycle,
     * 4 COM/USB AI-linked (serial `DD2` splits), 5 COM/USB unlinked (`DD4`).
     * Requires AI ON. Per-model restrictions live in the models table.
     */
    fun setDd0(code: Int): String? = if (code in 0..5) "DD0$code;" else null

    private fun hexVal(c: Char): Int = when (c) {
        in '0'..'9' -> c - '0'
        in 'A'..'F' -> c - 'A' + 10
        in 'a'..'f' -> c - 'a' + 10
        else -> -1
    }

    /**
     * Decode `2 * count` ASCII hex digits into `count` bins, two digits per
     * bin high nibble first, clamped at the display floor. Null on any
     * non-hex digit.
     */
    private fun hexBins(s: String, count: Int): ByteArray? {
        if (s.length != 2 * count) return null
        val out = ByteArray(count)
        for (i in 0 until count) {
            val hi = hexVal(s[2 * i])
            val lo = hexVal(s[2 * i + 1])
            if (hi < 0 || lo < 0) return null
            out[i] = ((hi shl 4) or lo).coerceAtMost(SCOPE_LEVEL_FLOOR).toByte()
        }
        return out
    }

    /**
     * Parse a LAN `##DD2` scope line: exactly the 5-char prefix plus 1280 hex
     * digits (the terminator already stripped by the splitter). 640 bins, two
     * digits each high nibble first, first pair = lowest frequency, values
     * clamped at 0x8C. Any wrong length or non-hex digit rejects the frame.
     */
    fun parseLanDd2(seg: String): ByteArray? =
        seg.afterPrefix("##DD2")?.let { hexBins(it, SCOPE_BINS) }

    /** One serial `DD2` split: (split index 00-31, 20 bins). */
    fun parseSerialDd2(seg: String): Pair<Int, ByteArray>? {
        val rest = seg.afterPrefix("DD2") ?: return null
        if (rest.length != 42) return null
        val split = parseDigits(rest.substring(0, 2), 2)?.toInt() ?: return null
        if (split > 31) return null
        val bins = hexBins(rest.substring(2), 20) ?: return null
        return Pair(split, bins)
    }

    /**
     * Map raw scope bins to dB for the spectrum plane: 0 is the 0 dB top of
     * the scale, 0x8C (140) the -100 dB floor. The scale is treated as linear
     * between those two documented endpoints; the linearity in between is not
     * confirmed by the command reference.
     */
    fun binsToDb(bins: ByteArray): FloatArray {
        val out = FloatArray(bins.size)
        for (i in bins.indices) {
            val v = (bins[i].toInt() and 0xFF).coerceAtMost(SCOPE_LEVEL_FLOOR).toFloat()
            out[i] = -v * (100.0f / SCOPE_LEVEL_FLOOR)
        }
        return out
    }

    /**
     * Reassembles the 32 serial `DD2` splits (20 bins each) into one 640-bin
     * line. Split 00 always restarts a line; any out-of-sequence split drops
     * the pending line rather than delivering bins on the wrong axis.
     */
    class SerialScopeAssembler {
        private val bins = ByteArray(SCOPE_BINS)
        private var have = 0
        private var next = 0

        /** Lines discarded for arriving out of sequence. */
        var dropped: Long = 0
            private set

        /**
         * Feed one parsed split; returns the finished 640-bin line when
         * split 31 completes it.
         */
        fun push(split: Int, splitBins: ByteArray): ByteArray? {
            if (split > 31 || split < 0 || splitBins.size != 20) {
                abandon()
                return null
            }
            if (split == 0) {
                abandon()
            } else if (split != next) {
                abandon()
                return null
            }
            System.arraycopy(splitBins, 0, bins, have, 20)
            have += 20
            next = split + 1
            if (split == 31) {
                next = 0
                have = 0
                return bins.copyOf()
            }
            return null
        }

        private fun abandon() {
            if (have != 0) {
                dropped++
                have = 0
            }
            next = 0
        }
    }

    /** How the bandscope places its edges. */
    enum class BandscopeMode { CENTER, FIXED, AUTO_SCROLL }

    private fun bandscopeMode(d: Int): BandscopeMode? = when (d) {
        0 -> BandscopeMode.CENTER
        1 -> BandscopeMode.FIXED
        2 -> BandscopeMode.AUTO_SCROLL
        else -> null
    }

    fun getBs3(): String = "BS3;"

    fun parseBs3(seg: String): BandscopeMode? =
        seg.afterPrefix("BS3")?.let { parseDigits(it, 1) }?.let { bandscopeMode(it.toInt()) }

    /** Centre-mode span table indexed by the BS4 code. */
    val BS4_SPANS_HZ = longArrayOf(
        5_000, 10_000, 25_000, 50_000, 100_000, 200_000, 500_000,
    )

    fun getBs4(): String = "BS4;"

    fun setBs4(code: Int): String? =
        if (code in BS4_SPANS_HZ.indices) "BS4$code;" else null

    /** Parse a `BS4` answer into its span code. */
    fun parseBs4(seg: String): Int? {
        val c = seg.afterPrefix("BS4")?.let { parseDigits(it, 1) }?.toInt() ?: return null
        return if (c in BS4_SPANS_HZ.indices) c else null
    }

    fun bs4SpanHz(code: Int): Long? = BS4_SPANS_HZ.getOrNull(code)

    fun getBsm0(): String = "BSM0;"

    /** Parse the fixed/scroll edge answer: `BSM0` + two 8-digit Hz limits. */
    fun parseBsm0(seg: String): Pair<Long, Long>? {
        val rest = seg.afterPrefix("BSM0") ?: return null
        if (rest.length != 16) return null
        val low = parseDigits(rest.substring(0, 8), 8) ?: return null
        val high = parseDigits(rest.substring(8), 8) ?: return null
        return if (high > low) Pair(low, high) else null
    }

    fun getBso(): String = "BSO;"

    /** EXPAND on/off. */
    fun parseBso(seg: String): Boolean? =
        seg.afterPrefix("BSO")?.let { parseDigits(it, 1) }?.let { it == 1L }

    fun getBsc(): String = "BSC;"

    /** Reference level, 000-140. */
    fun parseBsc(seg: String): Int? {
        val v = seg.afterPrefix("BSC")?.let { parseDigits(it, 3) } ?: return null
        return if (v <= 140) v.toInt() else null
    }

    /** Everything DD4 answer 1 says about the sweep that follows. */
    data class Dd4Header(
        val mode: BandscopeMode,
        /** Centre mode: span Hz. Fixed/scroll: low edge Hz. */
        val field1Hz: Long,
        /** Centre mode: centre Hz. Fixed/scroll: high edge Hz. */
        val field2Hz: Long,
        /** The wanted signal left the scoped range. */
        val outOfRange: Boolean,
    ) {
        /** The sweep edges this header describes. */
        fun edges(expand: Boolean): Pair<Long, Long>? = when (mode) {
            BandscopeMode.CENTER ->
                deriveEdges(mode, field2Hz, field1Hz, null, expand)
            else ->
                deriveEdges(mode, 0, null, Pair(field1Hz, field2Hz), expand)
        }
    }

    /**
     * Parse the self-describing `DD4` answer 1: scope mode digit, two
     * 11-digit Hz fields (centre mode: span then centre; fixed/scroll: low
     * then high edge) and the out-of-range flag.
     */
    fun parseDd4Header(seg: String): Dd4Header? {
        val rest = seg.afterPrefix("DD4") ?: return null
        if (rest.length != 24) return null
        val mode = parseDigits(rest.substring(0, 1), 1)
            ?.let { bandscopeMode(it.toInt()) } ?: return null
        val field1 = parseDigits(rest.substring(1, 12), 11) ?: return null
        val field2 = parseDigits(rest.substring(12, 23), 11) ?: return null
        val outOfRange = parseDigits(rest.substring(23), 1)?.let { it != 0L } ?: return null
        return Dd4Header(mode, field1, field2, outOfRange)
    }

    /**
     * Factor applied to the displayed span when EXPAND is on. The reference
     * documents the streamed data as covering 2-3x the displayed span
     * depending on span; the exact per-span factor is not published, so the
     * lower bound is used and the axis error above it is accepted until
     * measured on hardware.
     */
    const val EXPAND_FACTOR = 2L

    /**
     * Derive the scope edges from the bandscope state. Centre mode needs the
     * centre (the tuned frequency) and the BS4 span; fixed and auto-scroll
     * need the BSM0 limits. With EXPAND on the streamed data covers a wider
     * range than the display; the derived span widens by [EXPAND_FACTOR]
     * around the same centre.
     */
    fun deriveEdges(
        mode: BandscopeMode,
        centerHz: Long,
        spanHz: Long?,
        limits: Pair<Long, Long>?,
        expand: Boolean,
    ): Pair<Long, Long>? {
        val center: Long
        var span: Long
        when (mode) {
            BandscopeMode.CENTER -> {
                span = spanHz ?: return null
                if (span <= 0 || centerHz <= 0) return null
                center = centerHz
            }
            BandscopeMode.FIXED, BandscopeMode.AUTO_SCROLL -> {
                val (low, high) = limits ?: return null
                if (high <= low) return null
                center = low + (high - low) / 2
                span = high - low
            }
        }
        if (expand) span *= EXPAND_FACTOR
        return Pair(center - span / 2, center + span / 2)
    }
}
